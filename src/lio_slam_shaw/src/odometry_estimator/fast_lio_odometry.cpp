#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

#include <omp.h>

#include <algorithm>
#include <iterator>
#include <numeric>
#include <stdexcept>

namespace lio_slam_shaw::odometry_estimator {

static inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return S;
}

// Linear interpolation of IMU between two bracketing samples at target timestamp.
static core::ImuData interpolateImu(const core::ImuData& before, const core::ImuData& after,
                                    const core::Timestamp& target) {
    const double dt = core::getDeltaSec(before.timestamp, after.timestamp);
    const double alpha = (dt > 0.0) ? core::getDeltaSec(before.timestamp, target) / dt : 0.0;
    core::ImuData interp;
    interp.timestamp = target;
    interp.acc = before.acc + alpha * (after.acc - before.acc);
    interp.gyr = before.gyr + alpha * (after.gyr - before.gyr);
    return interp;
}

FastLioOdometry::FastLioOdometry(core::IMapBuilder::SharedPtr map_builder,
                                 const Eigen::Isometry3d& T_base_lidar,
                                 const Eigen::Isometry3d& T_base_imu,
                                 const FastLioOdometryParams& params)
    : params_(params), T_base_lidar_(T_base_lidar) {
    map_builder_ = std::dynamic_pointer_cast<map_builder::IkdTreeMapBuilder>(map_builder);
    if (!map_builder_) {
        throw std::invalid_argument("FastLioOdometry requires an IkdTreeMapBuilder instance");
    }
    T_imu_lidar_ = T_base_imu.inverse() * T_base_lidar;
    prev_scan_time_ = core::Timestamp::min();
}

// ---------------------------------------------------------------------------
// IOdometryEstimator interface
// ---------------------------------------------------------------------------

void FastLioOdometry::feedImu(const core::ImuData& imu) {
    if (!bias_initialized_) {
        tryInitBias(imu);
    }

    {
        std::lock_guard<std::mutex> imu_lock(imu_buf_mutex_);
        imu_buf_.push_back(imu);

        // Trim stale IMU data (keep at most 10 seconds behind prev_scan_time_)
        constexpr auto kMaxLag = std::chrono::seconds(10);
        while (imu_buf_.size() > 1 &&
               (imu_buf_.front().timestamp + kMaxLag) < imu_buf_.back().timestamp) {
            imu_buf_.pop_front();
        }
    }

    // Don't propagate predicted states until bias is initialised — integration would use b=0.
    if (!bias_initialized_) return;

    {
        // Snapshot committed_state_ under its own lock first to avoid data race.
        // Lock order: committed → predicted (consistent with estimateWithFeatures).
        IeskfState committed_snapshot;
        {
            std::lock_guard<std::mutex> c_lock(committed_state_mutex_);
            committed_snapshot = committed_state_;
        }
        std::lock_guard<std::mutex> lock(predicted_states_mutex_);
        const IeskfState& last_state =
            predicted_states_.empty() ? committed_snapshot : predicted_states_.back();
        predicted_states_.push_back(predictStep(last_state, imu));
    }
}

FastLioOdometry::IeskfState FastLioOdometry::predictStep(const IeskfState& state,
                                                         const core::ImuData& imu) {
    double dt = core::getDeltaSec(state.timestamp, imu.timestamp);
    if (dt <= 0.0) return state;

    const Eigen::Vector3d omega = imu.gyr - state.b_g;
    const Eigen::Vector3d acc = imu.acc - state.b_a;

    // Midpoint integration
    const Eigen::Vector3d rot_half = omega * (dt * 0.5);
    const double angle_half = rot_half.norm();
    const Eigen::Matrix3d dR_half =
        angle_half < 1e-10
            ? (Eigen::Matrix3d::Identity() + skew(rot_half))
            : Eigen::AngleAxisd(angle_half, rot_half / angle_half).toRotationMatrix();
    const Eigen::Vector3d a_world = (state.R * dR_half) * acc + gravity_;

    const Eigen::Vector3d rot_full = omega * dt;
    const double angle_full = rot_full.norm();
    const Eigen::Matrix3d dR_full =
        angle_full < 1e-10
            ? (Eigen::Matrix3d::Identity() + skew(rot_full))
            : Eigen::AngleAxisd(angle_full, rot_full / angle_full).toRotationMatrix();

    IeskfState predicted = state;
    predicted.timestamp = imu.timestamp;
    predicted.angular_vel = omega;
    predicted.R = state.R * dR_full;
    predicted.v = state.v + a_world * dt;
    predicted.p = state.p + state.v * dt + 0.5 * a_world * dt * dt;
    predicted.b_g = state.b_g;
    predicted.b_a = state.b_a;
    // Note: P is not propagated in predictStep (high-freq odom path; P updated in propagateStep)
    return predicted;
}

FastLioOdometry::IeskfState FastLioOdometry::propagateStep(const IeskfState& state,
                                                           const core::ImuData& imu) {
    IeskfState next = predictStep(state, imu);

    const double dt = core::getDeltaSec(state.timestamp, imu.timestamp);
    if (dt <= 0.0) return next;

    const Eigen::Vector3d omega = imu.gyr - state.b_g;
    const Eigen::Vector3d acc = imu.acc - state.b_a;

    // State ordering: [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:18)]
    // Matches Fast-LIO paper F matrix (b_a before b_g per image convention)
    // F (18×18) — discrete-time linearisation (first-order Euler)
    Eigen::Matrix<double, 18, 18> F = Eigen::Matrix<double, 18, 18>::Identity();
    // δp: ṗ = v  →  δp_{k+1} += δv dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;  // ∂δp/∂δv
    // δv: v̇ = R(a)+g  →  δv_{k+1} += -R[a]×δθ dt − R δb_a dt + δg dt
    F.block<3, 3>(3, 6) = -state.R * skew(acc) * dt;          // ∂δv/∂δθ
    F.block<3, 3>(3, 9) = -state.R * dt;                      // ∂δv/∂δb_a
    F.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;  // ∂δv/∂δg
    // δθ: Ṙ = R[ω]×  →  δθ_{k+1} = (I−[ω]×dt) δθ − δb_g dt
    F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(omega) * dt;  // ∂δθ/∂δθ
    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;              // ∂δθ/∂δb_g
    // b_a, b_g, g: identity (random walk driven by Q)

    // G (18×12) = F_i from paper; noise input cols = [v_i/acc(0:3), θ_i/gyr(3:6),
    // A_i/acc_bias(6:9), Ω_i/gyr_bias(9:12)] Q_i = diag(V_i, Θ_i, A_i, Ω_i) — same column order
    Eigen::Matrix<double, 18, 12> G = Eigen::Matrix<double, 18, 12>::Zero();
    G.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();   // δv ← acc noise (V_i)
    G.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();   // δθ ← gyr noise (Θ_i)
    G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();   // δb_a ← acc_bias walk (A_i)
    G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();  // δb_g ← gyr_bias walk (Ω_i)

    // Q_i: discrete-time noise covariance (matches paper)
    //   measurement noise (gyr/acc):  σ² Δt²  (V_i, Θ_i)
    //   bias random walk:             σ² Δt   (A_i, Ω_i)
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    const double dt2 = dt * dt;
    Qi.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * params_.acc_noise * params_.acc_noise * dt2;  // V_i
    Qi.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * params_.gyr_noise * params_.gyr_noise * dt2;  // Θ_i
    Qi.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * params_.acc_bias_noise * params_.acc_bias_noise * dt;  // A_i
    Qi.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * params_.gyr_bias_noise * params_.gyr_bias_noise * dt;  // Ω_i

    next.P = F * state.P * F.transpose() + G * Qi * G.transpose();
    return next;
}

core::OdometryResult FastLioOdometry::estimateWithFeatures(const core::FeatureSet& features,
                                                           core::Timestamp lidar_time_start) {
    // Cannot run update before IMU biases are initialized.
    if (!bias_initialized_) {
        core::OdometryResult empty;
        empty.matched_in_map.is_converged = false;
        empty.matched_in_odom.is_converged = false;
        return empty;
    }

    // 1. Collect IMU batch [prev_scan_time_, lidar_time_start] with interpolated endpoints.
    std::vector<core::ImuData> imu_batch;
    {
        std::lock_guard<std::mutex> lock(imu_buf_mutex_);

        // lower_bound comp: (element, value) → element.timestamp < value
        // upper_bound comp: (value, element) → value < element.timestamp
        auto lb_cmp = [](const core::ImuData& a, const core::Timestamp& t) {
            return a.timestamp < t;
        };
        auto ub_cmp = [](const core::Timestamp& t, const core::ImuData& a) {
            return t < a.timestamp;
        };

        // Note: no start-boundary interpolation needed — committed_state_.timestamp already
        // equals prev_scan_time_, so a virtual sample there gives dt = 0 (no-op integrate).

        // Collect all samples in (prev_scan_time_, lidar_time_start).
        const auto start_it =
            (prev_scan_time_ == core::Timestamp::min())
                ? imu_buf_.begin()
                : std::upper_bound(imu_buf_.begin(), imu_buf_.end(), prev_scan_time_, ub_cmp);
        const auto end_it =
            std::lower_bound(imu_buf_.begin(), imu_buf_.end(), lidar_time_start, lb_cmp);
        // end_it points to the first sample >= lidar_time_start; copy everything before it.
        imu_batch.assign(start_it, end_it);

        // Interpolate end boundary at lidar_time_start.
        // after_end_it = first sample >= lidar_time_start (= end_it).
        // before = prev(end_it) if available.
        const auto after_end_it = end_it;
        const bool has_after = (after_end_it != imu_buf_.end());
        const bool has_before = (after_end_it != imu_buf_.begin());
        if (has_before && has_after) {
            imu_batch.push_back(
                interpolateImu(*std::prev(after_end_it), *after_end_it, lidar_time_start));
        } else if (has_before) {
            // No sample after lidar_time_start yet — extrapolate by repeating last sample.
            core::ImuData extrap = *std::prev(after_end_it);
            extrap.timestamp = lidar_time_start;
            imu_batch.push_back(extrap);
        }

        while (imu_buf_.size() > 1 && imu_buf_.front().timestamp < prev_scan_time_) {
            imu_buf_.pop_front();
        }
    }

    // 2. Forward propagation (IMU integration + covariance propagation)
    // Propagates committed_state_ from prev_scan_time to lidar_time_start.
    // TODO: replace state_ with committed_state_ once dual-state refactor is complete.

    IeskfState reprop_start;
    {
        std::lock_guard<std::mutex> lock(committed_state_mutex_);
        IeskfState propagated = committed_state_;

        if (!imu_batch.empty()) {
            for (const auto& imu : imu_batch) {
                propagated = propagateStep(propagated, imu);
            }
        }

        // 3. iEKF iterated update with point-to-plane residuals
        committed_state_ = iteratedUpdate(propagated, features);
        reprop_start = committed_state_;
    }
    std::vector<core::ImuData> reprop_batch;
    {
        // Lock both mutexes in the same order as feedImu (predicted → imu) to avoid deadlock.
        // Holding both locks makes the reprop + predicted_states swap atomic:
        // feedImu either completes before this block or waits until after.
        std::lock_guard<std::mutex> pred_lock(predicted_states_mutex_);
        std::lock_guard<std::mutex> imu_lock(imu_buf_mutex_);

        reprop_batch.clear();
        for (const auto& imu : imu_buf_) {
            if (imu.timestamp > lidar_time_start) reprop_batch.push_back(imu);
        }

        IeskfState reproped = reprop_start;
        std::deque<IeskfState> reproped_states;
        for (const auto& imu : reprop_batch) {
            reproped = propagateStep(reproped, imu);
            reproped_states.push_back(reproped);
        }

        // Replace predicted_states: keep only entries newer than reprop_batch coverage
        while (!predicted_states_.empty() && !reprop_batch.empty() &&
               predicted_states_.front().timestamp <= reprop_batch.back().timestamp) {
            predicted_states_.pop_front();
        }
        predicted_states_.insert(predicted_states_.begin(), reproped_states.begin(),
                                 reproped_states.end());
    }

    prev_scan_time_ = lidar_time_start;

    // 5. Build result — read committed_state_ while still holding committed_state_mutex_
    //    (released at end of the outer lock scope below)
    core::OdometryResult result;
    {
        std::lock_guard<std::mutex> lock(committed_state_mutex_);
        Eigen::Isometry3d pose_map = Eigen::Isometry3d::Identity();
        pose_map.linear() = committed_state_.R;
        pose_map.translation() = committed_state_.p;

        core::ScanMatchResult matched_in_map;
        matched_in_map.pose = pose_map;
        matched_in_map.is_converged = true;

        core::ScanMatchResult matched_in_odom;
        matched_in_odom.pose = T_map_odom_.inverse() * pose_map;
        matched_in_odom.is_converged = true;

        core::NavState nav;
        nav.timestamp = lidar_time_start;
        nav.pose = matched_in_odom.pose;
        nav.linear_vel = committed_state_.v;
        nav.angular_vel = committed_state_.angular_vel;  // ω = gyr - b_g from last IMU step
        nav.acc_bias = committed_state_.b_a;
        nav.gyr_bias = committed_state_.b_g;

        result = core::OdometryResult{matched_in_map, matched_in_odom, nav};
    }
    return result;
}

FastLioOdometry::IeskfState FastLioOdometry::iteratedUpdate(const IeskfState& propagated,
                                                            const core::FeatureSet& features) {
    const auto cloud = features.raw_deskewed;
    if (!cloud || cloud->empty()) return propagated;

    // Convert propagated state to map frame; work in map frame throughout.
    IeskfState propagated_map = propagated;
    {
        Eigen::Isometry3d T_odom = Eigen::Isometry3d::Identity();
        T_odom.linear() = propagated_map.R;
        T_odom.translation() = propagated_map.p;
        const Eigen::Isometry3d T_world = T_map_odom_ * T_odom;
        propagated_map.R = T_world.linear();
        propagated_map.p = T_world.translation();
    }

    IeskfState result = propagated_map;
    const Eigen::Matrix<double, 18, 18> P_bar = propagated.P;  // propagated covariance (fixed)

    // Saved from last valid iteration for the final P update
    Eigen::MatrixXd K_last;
    Eigen::MatrixXd H_last;
    int last_valid_num = 0;

    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        const Eigen::Isometry3d T_world_lidar = [&] {
            Eigen::Isometry3d T;
            T.linear() = result.R;
            T.translation() = result.p;
            return T * T_imu_lidar_;
        }();
        const Eigen::Vector3d t_map_lidar = T_world_lidar.translation();

        // --- KNN + plane fitting (parallelised) ---
        std::vector<NearestPlaneResult> nearest_planes(cloud->size());
        const int n_pts = static_cast<int>(cloud->size());
#pragma omp parallel for schedule(static)
        for (int i = 0; i < n_pts; ++i) {
            nearest_planes[i] = queryNearestPlane((*cloud)[i], T_world_lidar);
        }

        // --- Assemble H, r ---
        Eigen::MatrixXd H_raw(n_pts, 18);
        Eigen::VectorXd r_raw(n_pts);
        int valid_num = 0;

        for (int i = 0; i < n_pts; ++i) {
            if (!nearest_planes[i].valid) continue;
            const Eigen::Vector3d p_lidar((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
            const auto res = buildPointResidual(nearest_planes[i], p_lidar, t_map_lidar, result.R);
            if (!res.valid) continue;
            H_raw.row(valid_num) = res.H;
            r_raw(valid_num) = res.r;
            ++valid_num;
        }

        if (valid_num < 6) break;  // under-constrained

        const auto H = H_raw.topRows(valid_num);
        const auto r = r_raw.head(valid_num);

        // --- Prior offset: dx_prior = result ⊞⁻¹ propagated_map ---
        Eigen::Matrix<double, 18, 1> dx_prior = Eigen::Matrix<double, 18, 1>::Zero();
        dx_prior.segment<3>(0) = result.p - propagated_map.p;
        dx_prior.segment<3>(3) = result.v - propagated_map.v;
        {
            // Log map: SO(3) → so(3)
            const Eigen::AngleAxisd aa(propagated_map.R.transpose() * result.R);
            dx_prior.segment<3>(6) = aa.angle() < 1e-10 ? Eigen::Vector3d::Zero()
                                                        : Eigen::Vector3d(aa.angle() * aa.axis());
        }
        dx_prior.segment<3>(9) = result.b_a - propagated_map.b_a;
        dx_prior.segment<3>(12) = result.b_g - propagated_map.b_g;
        // dx_prior[15:18] = 0 (gravity not estimated)

        // --- Kalman gain (using propagated P_bar, NOT iteratively shrunk P) ---
        const Eigen::MatrixXd S =
            H * P_bar * H.transpose() +
            params_.measurement_noise * Eigen::MatrixXd::Identity(valid_num, valid_num);
        const Eigen::MatrixXd K = P_bar * H.transpose() * S.inverse();

        // iEKF update (derived from MAP optimisation, innovation convention):
        //   dx_total^(κ+1) = K * (r + H * dx_prior)
        //   x̂^(κ+1) = x̄ ⊞ dx_total^(κ+1)
        const Eigen::VectorXd dx_total = K * (r + H * dx_prior);

        result = propagated_map;  // reset to prior
        result.p += dx_total.segment<3>(0);
        result.v += dx_total.segment<3>(3);
        result.b_a += dx_total.segment<3>(9);
        result.b_g += dx_total.segment<3>(12);

        const Eigen::Vector3d dtheta = dx_total.segment<3>(6);
        const double angle = dtheta.norm();
        const Eigen::Matrix3d dR =
            angle < 1e-10 ? (Eigen::Matrix3d::Identity() + skew(dtheta))
                          : Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix();
        result.R = propagated_map.R * dR;

        // Save for final P update
        K_last = K;
        H_last = H;
        last_valid_num = valid_num;

        // Convergence: ||dx_total − dx_prior|| = incremental change
        if ((dx_total - dx_prior).norm() < params_.state_converge_threshold) break;
    }

    // --- Covariance update (Joseph form, done ONCE after convergence) ---
    if (last_valid_num >= 6) {
        const Eigen::Matrix<double, 18, 18> I18 = Eigen::Matrix<double, 18, 18>::Identity();
        const Eigen::MatrixXd IKH = I18 - K_last * H_last;
        result.P = IKH * P_bar * IKH.transpose() +
                   K_last *
                       (params_.measurement_noise *
                        Eigen::MatrixXd::Identity(last_valid_num, last_valid_num)) *
                       K_last.transpose();
    } else {
        result.P = P_bar;
    }

    // Convert result back to odom frame
    {
        Eigen::Isometry3d T_world_result = Eigen::Isometry3d::Identity();
        T_world_result.linear() = result.R;
        T_world_result.translation() = result.p;
        const Eigen::Isometry3d T_odom_result = T_map_odom_.inverse() * T_world_result;
        result.R = T_odom_result.linear();
        result.p = T_odom_result.translation();
    }

    return result;
}

FastLioOdometry::NearestPlaneResult FastLioOdometry::queryNearestPlane(
    const core::PointXYZIRT& pt_lidar, const Eigen::Isometry3d& T_world_lidar) const {
    const Eigen::Matrix3d R = T_world_lidar.linear();
    const Eigen::Vector3d t = T_world_lidar.translation();

    core::PointXYZIRT q = pt_lidar;
    q.x = static_cast<float>(R(0, 0) * pt_lidar.x + R(0, 1) * pt_lidar.y + R(0, 2) * pt_lidar.z +
                             t.x());
    q.y = static_cast<float>(R(1, 0) * pt_lidar.x + R(1, 1) * pt_lidar.y + R(1, 2) * pt_lidar.z +
                             t.y());
    q.z = static_cast<float>(R(2, 0) * pt_lidar.x + R(2, 1) * pt_lidar.y + R(2, 2) * pt_lidar.z +
                             t.z());

    thread_local std::vector<core::PointXYZIRT> neighbors;
    thread_local std::vector<float> distances;
    neighbors.clear();
    distances.clear();

    if (!map_builder_->searchKNearestPoints(q, params_.num_nearest_neighbors, params_.search_radius,
                                            neighbors, distances)) {
        return {false, {}, {}, {}};
    }
    if (static_cast<int>(neighbors.size()) < params_.min_plane_points) {
        return {false, {}, {}, {}};
    }
    return fitPlane(neighbors, Eigen::Vector3d(q.x, q.y, q.z));
}

FastLioOdometry::NearestPlaneResult FastLioOdometry::fitPlane(
    const std::vector<lio_slam_shaw::core::PointXYZIRT>& neighbors,
    const Eigen::Vector3d& query_point_in_map) const {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) centroid += Eigen::Vector3d(p.x, p.y, p.z);
    centroid /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d dp = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        cov += dp * dp.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const auto& eigenvalues = solver.eigenvalues();

    if (eigenvalues(0) > params_.min_plane_eigenvalue_ratio * eigenvalues(2)) {
        return {false, {}, {}, {}};
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0);

    return NearestPlaneResult{
        /*valid=*/true,
        /*point_in_map=*/query_point_in_map,
        /*normal=*/normal,
        /*centroid=*/centroid,
    };
}

FastLioOdometry::PointResidual FastLioOdometry::buildPointResidual(
    const NearestPlaneResult& plane, const Eigen::Vector3d& p_lidar,
    const Eigen::Vector3d& sensor_origin_in_map, const Eigen::Matrix3d& R_map_body) const {
    // Flip normal toward sensor
    Eigen::Vector3d normal = plane.normal;
    if (normal.dot(sensor_origin_in_map - plane.point_in_map) < 0) normal = -normal;

    // Signed point-to-plane distance (prediction = h(x̂))
    const double dist = normal.dot(plane.point_in_map - plane.centroid);
    if (std::abs(dist) >= params_.max_point_to_plane_distance) return {};

    // Jacobian: state ordering [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:18)]
    // Right perturbation on R_map_body → point must be in body (base) frame
    const Eigen::Vector3d p_imu = T_imu_lidar_ * p_lidar;
    PointResidual res;
    res.valid = true;
    res.H.setZero();
    res.H.block<1, 3>(0, 0) = normal.transpose();                              // ∂r/∂δp
    res.H.block<1, 3>(0, 6) = -normal.transpose() * R_map_body * skew(p_imu);  // ∂r/∂δθ
    // Innovation convention: r = z - h(x̂) = 0 - dist  (point-on-plane → z = 0)
    res.r = -dist;
    return res;
}

void FastLioOdometry::tryInitBias(const core::ImuData& imu) {
    if (bias_initialized_) return;

    static_imu_buf_.push_back(imu);
    if (static_imu_buf_.size() < kMinStaticSamples) return;
    if (static_imu_buf_.size() > kMaxStaticSamples) {
        bias_initialized_ = true;  // cap reached → use what we have
    }

    if (!bias_initialized_ && static_imu_buf_.size() < kMaxStaticSamples) return;

    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d mean_gyr = Eigen::Vector3d::Zero();
    for (const auto& s : static_imu_buf_) {
        mean_acc += s.acc;
        mean_gyr += s.gyr;
    }
    mean_acc /= static_cast<double>(static_imu_buf_.size());
    mean_gyr /= static_cast<double>(static_imu_buf_.size());

    {
        std::lock_guard<std::mutex> lock(committed_state_mutex_);
        // Gyro bias = mean gyro reading (robot is static)
        committed_state_.b_g = mean_gyr;
        // Accel bias: mean_acc should equal R^T * g when static.
        // Assume initial pose is level → gravity along -Z in body frame.
        committed_state_.b_a = mean_acc - Eigen::Vector3d(0.0, 0.0, kGravity);
        // Seed the timestamp so the first predictStep sees a valid dt.
        committed_state_.timestamp = imu.timestamp;
    }

    bias_initialized_ = true;
    static_imu_buf_.clear();
    static_imu_buf_.shrink_to_fit();
}

core::NavState FastLioOdometry::getLatestState() const {
    // Snapshot committed_state_ first so we have a safe fallback without holding two locks.
    IeskfState committed_snapshot;
    {
        std::lock_guard<std::mutex> c_lock(committed_state_mutex_);
        committed_snapshot = committed_state_;
    }
    const IeskfState* s = &committed_snapshot;
    IeskfState predicted_snapshot;
    {
        std::lock_guard<std::mutex> lock(predicted_states_mutex_);
        if (!predicted_states_.empty()) {
            predicted_snapshot = predicted_states_.back();
            s = &predicted_snapshot;
        }
    }
    core::NavState nav;
    nav.pose.linear() = s->R;
    nav.pose.translation() = s->p;
    nav.linear_vel = s->v;
    nav.angular_vel = s->angular_vel;  // ω = gyr - b_g, updated every IMU step
    nav.acc_bias = s->b_a;
    nav.gyr_bias = s->b_g;
    return nav;
}

std::vector<core::NavState> FastLioOdometry::getNavStateQueueSnapshot() const {
    // Snapshot committed_state_ first (lock order: committed → predicted).
    IeskfState committed_snapshot;
    {
        std::lock_guard<std::mutex> c_lock(committed_state_mutex_);
        committed_snapshot = committed_state_;
    }

    auto toNavState = [](const IeskfState& s) {
        core::NavState nav;
        nav.timestamp = s.timestamp;
        nav.pose.linear() = s.R;
        nav.pose.translation() = s.p;
        nav.linear_vel = s.v;
        nav.angular_vel = s.angular_vel;
        nav.acc_bias = s.b_a;
        nav.gyr_bias = s.b_g;
        return nav;
    };

    std::vector<core::NavState> result;
    result.push_back(toNavState(committed_snapshot));

    {
        std::lock_guard<std::mutex> lock(predicted_states_mutex_);
        result.reserve(1 + predicted_states_.size());
        for (const auto& ps : predicted_states_) {
            result.push_back(toNavState(ps));
        }
    }
    return result;
}

void FastLioOdometry::setMapToOdomTransform(const Eigen::Isometry3d& T_map_odom) {
    std::lock_guard<std::mutex> lock(committed_state_mutex_);
    T_map_odom_ = T_map_odom;
}

}  // namespace lio_slam_shaw::odometry_estimator
