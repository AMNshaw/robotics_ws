#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

#include <omp.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
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
    {
        std::lock_guard<std::mutex> imu_lock(imu_buf_mutex_);
        imu_buf_.push_back(imu);

        // Trim only IMU samples that the frontend has already consumed.
        // prev_scan_time_ marks the latest lidar frame processed by estimateWithFeatures.
        // Keeping everything after prev_scan_time_ ensures that even when the frontend
        // falls behind real-time, no needed IMU data is discarded.
        if (prev_scan_time_ != core::Timestamp::min()) {
            while (imu_buf_.size() > 2 && imu_buf_[1].timestamp <= prev_scan_time_) {
                imu_buf_.pop_front();
            }
        }
    }

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
    const Eigen::Vector3d a_world = (state.R * dR_half) * acc + state.gravity;

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
    predicted.gravity = state.gravity;
    predicted.gravity_basis = state.gravity_basis;
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

    // State ordering: [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:17)]
    // F (17×17) — discrete-time linearisation (first-order Euler)
    Eigen::Matrix<double, kStateDim, kStateDim> F =
        Eigen::Matrix<double, kStateDim, kStateDim>::Identity();
    // δp: ṗ = v  →  δp_{k+1} += δv dt
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;  // ∂δp/∂δv
    // δv: v̇ = R(a)+g  →  δv_{k+1} += -R[a]×δθ dt − R δb_a dt + B δg dt
    F.block<3, 3>(3, 6) = -state.R * skew(acc) * dt;  // ∂δv/∂δθ
    F.block<3, 3>(3, 9) = -state.R * dt;              // ∂δv/∂δb_a
    F.block<3, 2>(3, 15) = state.gravity_basis * dt;  // ∂δv/∂δg (3×2)
    // δθ: Ṙ = R[ω]×  →  δθ_{k+1} = (I−[ω]×dt) δθ − δb_g dt
    F.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() - skew(omega) * dt;  // ∂δθ/∂δθ
    F.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;              // ∂δθ/∂δb_g
    // b_a, b_g: identity (random walk)
    // δg: identity (carried forward, driven by process noise)

    // G (17×12) — noise input matrix.
    // Columns: [w_acc(0:3), w_gyr(3:6), w_ba(6:9), w_bg(9:12)]
    Eigen::Matrix<double, kStateDim, 12> G = Eigen::Matrix<double, kStateDim, 12>::Zero();
    G.block<3, 3>(3, 0) = -state.R;                      // δv ← acc noise (body→world)
    G.block<3, 3>(6, 3) = -Eigen::Matrix3d::Identity();  // δθ ← gyr noise
    G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();   // δb_a ← acc_bias walk
    G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();  // δb_g ← gyr_bias walk

    // Q_i: continuous-time PSD → discrete: Q_d = Q_c · Δt
    //   All terms use σ² Δt (standard discretisation of white noise PSD).
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * params_.acc_noise * params_.acc_noise * dt;
    Qi.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * params_.gyr_noise * params_.gyr_noise * dt;
    Qi.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * params_.acc_bias_noise * params_.acc_bias_noise * dt;
    Qi.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * params_.gyr_bias_noise * params_.gyr_bias_noise * dt;

    next.P = F * state.P * F.transpose() + G * Qi * G.transpose();
    return next;
}

core::OdometryResult FastLioOdometry::estimateWithFeatures(const core::FeatureSet& features,
                                                           core::Timestamp lidar_time_start) {
    auto toSec = [](const core::Timestamp& t) {
        return std::chrono::duration<double>(t.time_since_epoch()).count();
    };

    using SteadyClock = std::chrono::steady_clock;
    auto t_section_start = SteadyClock::now();
    auto t_prev = t_section_start;
    auto elapsedMs = [&t_prev]() {
        auto now = SteadyClock::now();
        double ms = std::chrono::duration<double, std::milli>(now - t_prev).count();
        t_prev = now;
        return ms;
    };

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
    (void)elapsedMs();  // imu_batch_collect (silent)

    // Guard 1: no IMU data → skip (stale lidar frame, back-to-back processing)
    if (imu_batch.empty() && prev_scan_time_ != core::Timestamp::min()) {
        std::clog << "[FastLIO] SKIP: no IMU data between scans (stale lidar frame?)" << std::endl;
        prev_scan_time_ = lidar_time_start;
        core::OdometryResult empty;
        empty.matched_in_map.is_converged = false;
        empty.matched_in_odom.is_converged = false;
        return empty;
    }

    // Log large IMU batches (e.g. after init or frame drop) — but do NOT trim.
    // Trimming the front creates a time gap between committed_state_.timestamp and
    // the first remaining sample, turning the first propagateStep into a single
    // giant dt that catastrophically corrupts position/velocity.  Integrating all
    // samples (each with small dt ≈ 5 ms) is cheap and preserves accuracy.
    if (imu_batch.size() > 200) {
        std::clog << "[FastLIO] Large IMU batch: " << imu_batch.size() << " samples ("
                  << (imu_batch.empty() ? 0.0
                                        : core::getDeltaSec(imu_batch.front().timestamp,
                                                            imu_batch.back().timestamp))
                  << "s)\n";
    }

    // Guard 2: time gap too large → skip (state would diverge during long open-loop IMU prop)
    constexpr double kMaxScanGapSec = 10.0;
    if (prev_scan_time_ != core::Timestamp::min()) {
        const double gap = core::getDeltaSec(prev_scan_time_, lidar_time_start);
        if (gap > kMaxScanGapSec) {
            std::clog << "[FastLIO] SKIP: scan gap " << gap << "s" << std::endl;
            prev_scan_time_ = lidar_time_start;
            {
                std::lock_guard<std::mutex> lock(committed_state_mutex_);
                committed_state_.timestamp = lidar_time_start;
            }
            core::OdometryResult empty;
            empty.matched_in_map.is_converged = false;
            empty.matched_in_odom.is_converged = false;
            return empty;
        }
    }

    (void)elapsedMs();  // guards (silent)

    // 2. Forward propagation (IMU integration + covariance propagation)
    IeskfState reprop_start;
    double t_iter_ms = 0.0;
    {
        std::lock_guard<std::mutex> lock(committed_state_mutex_);
        IeskfState propagated = committed_state_;

        if (!imu_batch.empty()) {
            for (const auto& imu : imu_batch) {
                propagated = propagateStep(propagated, imu);
            }
        }

        (void)elapsedMs();  // fwd_propagation (silent)

        // 3. iEKF iterated update with point-to-plane residuals
        committed_state_ = iteratedUpdate(propagated, features);
        reprop_start = committed_state_;
        t_iter_ms = elapsedMs();
    }
    std::vector<core::ImuData> reprop_batch;
    size_t reprop_count = 0;
    {
        // Lock both mutexes in the same order as feedImu (predicted → imu) to avoid deadlock.
        std::lock_guard<std::mutex> pred_lock(predicted_states_mutex_);
        std::lock_guard<std::mutex> imu_lock(imu_buf_mutex_);

        reprop_batch.clear();
        for (const auto& imu : imu_buf_) {
            if (imu.timestamp > lidar_time_start) reprop_batch.push_back(imu);
        }

        // Use predictStep (no covariance) — predicted states only need pose/vel for deskew
        IeskfState reproped = reprop_start;
        std::deque<IeskfState> reproped_states;
        for (const auto& imu : reprop_batch) {
            reproped = predictStep(reproped, imu);
            reproped_states.push_back(reproped);
        }
        reprop_count = reprop_batch.size();

        // Replace predicted_states: keep only entries newer than reprop_batch coverage
        while (!predicted_states_.empty() && !reprop_batch.empty() &&
               predicted_states_.front().timestamp <= reprop_batch.back().timestamp) {
            predicted_states_.pop_front();
        }
        predicted_states_.insert(predicted_states_.begin(), reproped_states.begin(),
                                 reproped_states.end());
        const double t_reprop_ms = elapsedMs();

        // Single summary line
        std::clog << "[FastLIO] iter=" << t_iter_ms << "ms reprop=" << t_reprop_ms
                  << "ms(n=" << reprop_count << ")"
                  << " p=" << reprop_start.p.transpose() << " b_a=" << reprop_start.b_a.transpose()
                  << '\n';
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
    if (!cloud || cloud->empty()) {
        return propagated;
    }

    // Skip scan matching when the map is empty (first frame after init).
    // Querying an unbuilt ikd-tree returns garbage results that corrupt the state.
    if (!map_builder_->isMapReady()) {
        std::clog << "[FastLIO] Map not ready — skipping iEKF update (first frame)\n";
        return propagated;
    }

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
    const Eigen::Matrix<double, kStateDim, kStateDim> P_bar =
        propagated.P;  // propagated covariance (fixed)

    // Posterior covariance from Woodbury: (P_bar^{-1} + H^T R^{-1} H)^{-1}
    Eigen::Matrix<double, kStateDim, kStateDim> P_posterior = P_bar;
    int last_valid_num = 0;

    using Clock = std::chrono::steady_clock;
    for (int iter = 0; iter < params_.max_iterations; ++iter) {
        const auto t_iter_start = Clock::now();
        const Eigen::Isometry3d T_world_lidar = [&] {
            Eigen::Isometry3d T;
            T.linear() = result.R;
            T.translation() = result.p;
            return T * T_imu_lidar_;
        }();
        const Eigen::Vector3d t_map_lidar = T_world_lidar.translation();

        // --- Prior offset: dx_prior = result ⊞⁻¹ propagated_map ---
        Eigen::Matrix<double, kStateDim, 1> dx_prior = Eigen::Matrix<double, kStateDim, 1>::Zero();
        dx_prior.segment<3>(0) = result.p - propagated_map.p;
        dx_prior.segment<3>(3) = result.v - propagated_map.v;
        {
            const Eigen::AngleAxisd aa(propagated_map.R.transpose() * result.R);
            dx_prior.segment<3>(6) = aa.angle() < 1e-10 ? Eigen::Vector3d::Zero()
                                                        : Eigen::Vector3d(aa.angle() * aa.axis());
        }
        dx_prior.segment<3>(9) = result.b_a - propagated_map.b_a;
        dx_prior.segment<3>(12) = result.b_g - propagated_map.b_g;
        {
            const Eigen::Vector3d dg_3d = result.gravity - propagated_map.gravity;
            dx_prior.segment<2>(15) = propagated_map.gravity_basis.transpose() * dg_3d;
        }

        // --- KNN + buildResidual + accumulate H^T*H, H^T*r (fused, parallelised) ---
        const int n_pts = static_cast<int>(cloud->size());
        const int num_threads = omp_get_max_threads();

        // Per-thread accumulators (cache-line padded)
        struct alignas(64) ThreadAccum {
            Eigen::Matrix<double, kStateDim, kStateDim> HtH =
                Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
            Eigen::Matrix<double, kStateDim, 1> Htr = Eigen::Matrix<double, kStateDim, 1>::Zero();
            int count = 0;
            double sum_r2 = 0.0;  // for diagnostics: RMS residual
        };
        std::vector<ThreadAccum> accums(num_threads);

#pragma omp parallel
        {
            const int tid = omp_get_thread_num();
            auto& acc = accums[tid];
#pragma omp for schedule(static)
            for (int i = 0; i < n_pts; ++i) {
                const auto plane = queryNearestPlane((*cloud)[i], T_world_lidar);
                if (!plane.valid) continue;
                const Eigen::Vector3d p_lidar((*cloud)[i].x, (*cloud)[i].y, (*cloud)[i].z);
                const auto res = buildPointResidual(plane, p_lidar, t_map_lidar, result.R);
                if (!res.valid) continue;
                // rank-1 accumulation: H_i is 1×17
                acc.HtH.noalias() += res.H.transpose() * res.H;
                acc.Htr.noalias() += res.H.transpose() * res.r;
                acc.sum_r2 += res.r * res.r;
                ++acc.count;
            }
        }

        // Reduce across threads
        Eigen::Matrix<double, kStateDim, kStateDim> HtH =
            Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
        Eigen::Matrix<double, kStateDim, 1> Htr = Eigen::Matrix<double, kStateDim, 1>::Zero();
        int valid_num = 0;
        double sum_r2 = 0.0;
        for (int t = 0; t < num_threads; ++t) {
            HtH.noalias() += accums[t].HtH;
            Htr.noalias() += accums[t].Htr;
            valid_num += accums[t].count;
            sum_r2 += accums[t].sum_r2;
        }
        const double rms_residual = valid_num > 0 ? std::sqrt(sum_r2 / valid_num) : 0.0;
        const auto t_knn = Clock::now();
        (void)t_knn;

        if (valid_num < 6) break;  // under-constrained

        // --- Kalman gain via Woodbury identity (17×17 inverse instead of N×N) ---
        // z = r_inv * H^T * (r + H * dx_prior) = r_inv * (Htr + HtH * dx_prior)
        const double r_inv = 1.0 / params_.measurement_noise;
        const Eigen::Matrix<double, kStateDim, kStateDim> HtRinvH = r_inv * HtH;
        const Eigen::Matrix<double, kStateDim, kStateDim> P_bar_inv_plus_HtRinvH =
            P_bar.ldlt().solve(Eigen::Matrix<double, kStateDim, kStateDim>::Identity()) + HtRinvH;
        const Eigen::Matrix<double, kStateDim, kStateDim> gain_lhs =
            P_bar_inv_plus_HtRinvH.ldlt().solve(
                Eigen::Matrix<double, kStateDim, kStateDim>::Identity());
        const Eigen::Matrix<double, kStateDim, 1> z = r_inv * (Htr + HtH * dx_prior);
        const Eigen::Matrix<double, kStateDim, 1> dx_total = gain_lhs * z;

        result = propagated_map;  // reset to prior
        result.p += dx_total.segment<3>(0);
        result.v += dx_total.segment<3>(3);
        result.b_a += dx_total.segment<3>(9);
        result.b_g += dx_total.segment<3>(12);

        // Clamp bias magnitudes to physically reasonable bounds.
        // MEMS acc bias is typically <0.1 m/s²; unconstrained estimation can
        // produce wildly wrong values when the map is still sparse (first few
        // frames) or when the motion profile doesn't fully excite bias.
        constexpr double kMaxAccBias = 0.1;   // m/s²
        constexpr double kMaxGyrBias = 0.05;  // rad/s
        result.b_a = result.b_a.cwiseMax(-kMaxAccBias).cwiseMin(kMaxAccBias);
        result.b_g = result.b_g.cwiseMax(-kMaxGyrBias).cwiseMin(kMaxGyrBias);

        // Update gravity: g = g0 + B * δg, then re-project to sphere
        {
            const Eigen::Vector2d dg = dx_total.segment<2>(15);
            result.gravity =
                (propagated_map.gravity + propagated_map.gravity_basis * dg).normalized() *
                kGravity;
            result.updateGravityBasis();
        }

        const Eigen::Vector3d dtheta = dx_total.segment<3>(6);
        const double angle = dtheta.norm();
        const Eigen::Matrix3d dR =
            angle < 1e-10 ? (Eigen::Matrix3d::Identity() + skew(dtheta))
                          : Eigen::AngleAxisd(angle, dtheta / angle).toRotationMatrix();
        result.R = propagated_map.R * dR;

        // Save for final P update: gain_lhs = (P_bar^{-1} + H^T R^{-1} H)^{-1}
        // which IS the posterior covariance directly.
        P_posterior = gain_lhs;
        last_valid_num = valid_num;

        // Convergence: ||dx_total − dx_prior|| = incremental change
        const double delta = (dx_total - dx_prior).norm();
        if (iter == 0 || delta < params_.state_converge_threshold) {
            // Log first-iteration diagnostics: residual quality and correction magnitude
            std::clog << "[iEKF] it=" << iter << " valid=" << valid_num << "/" << n_pts
                      << " rms_r=" << rms_residual << " dp=" << dx_total.segment<3>(0).norm()
                      << " dv=" << dx_total.segment<3>(3).norm()
                      << " dba=" << dx_total.segment<3>(9).norm() << '\n';
        }
        if (delta < params_.state_converge_threshold) {
            break;
        }
    }

    // --- Covariance update ---
    // P_posterior = (P_bar^{-1} + H^T R^{-1} H)^{-1} is already the posterior covariance.
    if (last_valid_num >= 6) {
        result.P = P_posterior;
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

    // Zero-copy KNN: get pointers to thread_local buffers inside map_builder
    const map_builder::IkdTreeMapBuilder::PointVector* neighbors_ptr = nullptr;
    const std::vector<float>* distances_ptr = nullptr;

    if (!map_builder_->searchKNearestPointsDirect(q, params_.num_nearest_neighbors,
                                                  params_.search_radius, neighbors_ptr,
                                                  distances_ptr)) {
        return {false, {}, {}, {}};
    }
    if (static_cast<int>(neighbors_ptr->size()) < params_.min_plane_points) {
        return {false, {}, {}, {}};
    }
    return fitPlaneDirect(*neighbors_ptr, Eigen::Vector3d(q.x, q.y, q.z));
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

FastLioOdometry::NearestPlaneResult FastLioOdometry::fitPlaneDirect(
    const map_builder::IkdTreeMapBuilder::PointVector& neighbors,
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

    // Jacobian: state ordering [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:17)]
    // Right perturbation on R_map_body → point must be in body (base) frame
    const Eigen::Vector3d p_imu = T_imu_lidar_ * p_lidar;
    PointResidual res;
    res.valid = true;
    res.H.setZero();
    res.H.block<1, 3>(0, 0) = normal.transpose();                              // ∂r/∂δp
    res.H.block<1, 3>(0, 6) = -normal.transpose() * R_map_body * skew(p_imu);  // ∂r/∂δθ
    // ∂r/∂δg = 0 for point-to-plane (measurement doesn't depend on gravity directly)
    // Innovation convention: r = z - h(x̂) = 0 - dist  (point-on-plane → z = 0)
    res.r = -dist;
    return res;
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

void FastLioOdometry::setInitialState(const core::LioInitResult& init_result) {
    {
        std::lock_guard<std::mutex> lock(committed_state_mutex_);
        committed_state_.timestamp = init_result.timestamp;
        committed_state_.R = init_result.R;
        committed_state_.p = init_result.p;
        committed_state_.v = init_result.v;
        committed_state_.b_a = init_result.b_a;
        committed_state_.b_g = init_result.b_g;
        committed_state_.gravity = init_result.gravity;
        committed_state_.updateGravityBasis();
        // Reset covariance to small values
        committed_state_.P = Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
        committed_state_.P.diagonal().segment<3>(0).setConstant(1e-3);   // position
        committed_state_.P.diagonal().segment<3>(3).setConstant(1e-3);   // velocity
        committed_state_.P.diagonal().segment<3>(6).setConstant(1e-3);   // rotation
        committed_state_.P.diagonal().segment<3>(9).setConstant(1e-5);   // acc bias
        committed_state_.P.diagonal().segment<3>(12).setConstant(1e-4);  // gyr bias
        committed_state_.P.diagonal().segment<2>(15).setConstant(1e-2);  // gravity direction
    }
    gravity_ = init_result.gravity;
    {
        std::lock_guard<std::mutex> lock(imu_buf_mutex_);
        prev_scan_time_ = init_result.timestamp;
        imu_buf_.clear();
    }
    {
        std::lock_guard<std::mutex> lock(predicted_states_mutex_);
        predicted_states_.clear();
    }
    std::clog << "[FastLioOdom] setInitialState: p=(" << init_result.p.x() << ", "
              << init_result.p.y() << ", " << init_result.p.z() << ") g=("
              << init_result.gravity.x() << ", " << init_result.gravity.y() << ", "
              << init_result.gravity.z() << ")\n";
}

}  // namespace lio_slam_shaw::odometry_estimator
