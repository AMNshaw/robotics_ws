#include "lio_slam_shaw/initializer/sfm_lio_initializer.hpp"

#include <pcl/filters/voxel_grid.h>

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>

namespace lio_slam_shaw::initializer {

// ---------------------------------------------------------------------------
// Utility: linear interpolation of IMU between two bracketing samples
// ---------------------------------------------------------------------------
static core::ImuData interpolateImu(const core::ImuData& a, const core::ImuData& b,
                                    const core::Timestamp& t) {
    const double dt = core::getDeltaSec(a.timestamp, b.timestamp);
    const double alpha = (dt > 0.0) ? core::getDeltaSec(a.timestamp, t) / dt : 0.0;
    core::ImuData out;
    out.timestamp = t;
    out.acc = a.acc + alpha * (b.acc - a.acc);
    out.gyr = a.gyr + alpha * (b.gyr - a.gyr);
    return out;
}

// ---------------------------------------------------------------------------
// Utility: skew-symmetric matrix  (needed by expSO3)
// ---------------------------------------------------------------------------
static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),  //
        v.z(), 0, -v.x(),   //
        -v.y(), v.x(), 0;
    return m;
}

// ---------------------------------------------------------------------------
// SO(3) exponential map  (Rodrigues)
// ---------------------------------------------------------------------------
static Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    if (theta < 1e-10) return Eigen::Matrix3d::Identity();
    const Eigen::Vector3d a = phi / theta;
    const Eigen::Matrix3d ax = skew(a);
    return Eigen::Matrix3d::Identity() + std::sin(theta) * ax + (1.0 - std::cos(theta)) * (ax * ax);
}

// ===========================================================================
// Constructor
// ===========================================================================
SfmLioInitializer::SfmLioInitializer(
    const scan_matcher::IkdTreeScanMatcherParams& scan_matcher_params,
    const map_builder::IkdTreeMapBuilderParams& map_builder_params,
    const Eigen::Isometry3d& T_imu_lidar, const SfmLioInitializerParams& params)
    : params_(params),
      map_builder_params_(map_builder_params),
      scan_matcher_params_(scan_matcher_params),
      T_imu_lidar_(T_imu_lidar) {
    map_builder_ = std::make_shared<map_builder::IkdTreeMapBuilder>(map_builder_params);
    scan_matcher_ =
        std::make_shared<scan_matcher::IkdTreeScanMatcher>(map_builder_, scan_matcher_params);
    scan_matcher_->setLidarExtrinsics(T_imu_lidar_);
}

// ===========================================================================
// addImu — buffer every sample
// ===========================================================================
void SfmLioInitializer::addImu(const core::ImuData& imu) {
    if (ready_) return;
    imu_buf_.push_back(imu);
}

// ===========================================================================
// addScan — buffer only (O(1) with downsample)
// ===========================================================================
void SfmLioInitializer::addScan(const core::LidarData& lidar) {
    if (ready_) return;
    if (!lidar.cloud || lidar.cloud->empty()) return;

    // Downsample raw cloud
    core::PointCloudIRTPtr cloud = lidar.cloud;
    if (params_.voxel_leaf_size > 0.0f) {
        pcl::VoxelGrid<core::PointXYZIRT> voxel;
        voxel.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size,
                          params_.voxel_leaf_size);
        voxel.setInputCloud(lidar.cloud);
        cloud = std::make_shared<core::PointCloudIRT>();
        voxel.filter(*cloud);
    }

    scan_buf_.push_back({lidar.timestamp, cloud});
    std::clog << "[SfmInit] Buffered scan " << scan_buf_.size() << " (" << cloud->size()
              << " pts)\n";
}

// ===========================================================================
// hasEnoughData
// ===========================================================================
bool SfmLioInitializer::hasEnoughData() const {
    return !ready_ && static_cast<int>(scan_buf_.size()) >= params_.min_init_scans;
}

// ===========================================================================
// tryInitialize — batch scan matching + linear alignment (heavy computation)
// ===========================================================================
bool SfmLioInitializer::tryInitialize() {
    if (ready_) return true;
    if (static_cast<int>(scan_buf_.size()) < params_.min_init_scans) return false;

    std::clog << "[SfmInit] Running batch SFM on " << scan_buf_.size() << " scans...\n";

    for (size_t i = 0; i < scan_buf_.size(); ++i) {
        const auto& buf = scan_buf_[i];
        const auto& scan_time = buf.time;

        core::FeatureSet features;
        features.raw_deskewed = buf.cloud;

        if (frames_.empty()) {
            // First scan: origin
            Eigen::Isometry3d T_world_body = Eigen::Isometry3d::Identity();
            const Eigen::Isometry3d T_world_lidar = T_world_body * T_imu_lidar_;
            insertScanToMap(features, T_world_lidar);
            frames_.push_back({scan_time, T_world_body});
            std::clog << "[SfmInit] Frame 0 — origin\n";
            continue;
        }

        // Scan-to-map matching — constant-velocity extrapolation for initial guess
        core::NavState guess;
        guess.timestamp = scan_time;

        if (frames_.size() >= 2) {
            // Extrapolate from last two frames
            const auto& f0 = frames_[frames_.size() - 2];
            const auto& f1 = frames_[frames_.size() - 1];
            const double dt_prev = core::getDeltaSec(f0.time, f1.time);
            const double dt_curr = core::getDeltaSec(f1.time, scan_time);

            if (dt_prev > 1e-6 && dt_curr > 0.0) {
                const double ratio = dt_curr / dt_prev;
                const Eigen::Vector3d dp = f1.pose.translation() - f0.pose.translation();
                // Rotation: R_guess = R1 * exp(log(R0^T R1) * ratio)
                const Eigen::Matrix3d dR = f0.pose.rotation().transpose() * f1.pose.rotation();
                const Eigen::AngleAxisd aa(dR);

                guess.pose.linear() = f1.pose.rotation() * expSO3(aa.axis() * aa.angle() * ratio);
                guess.pose.translation() = f1.pose.translation() + dp * ratio;
            } else {
                guess.pose = f1.pose;
            }
        } else {
            guess.pose = frames_.back().pose;
        }

        const auto match_result = scan_matcher_->match(features, guess);
        const Eigen::Isometry3d T_world_body = match_result.pose;

        // Insert into map
        const Eigen::Isometry3d T_world_lidar = T_world_body * T_imu_lidar_;
        insertScanToMap(features, T_world_lidar);

        frames_.push_back({scan_time, T_world_body});
        std::clog << "[SfmInit] Frame " << frames_.size() - 1 << " — p=("
                  << T_world_body.translation().x() << ", " << T_world_body.translation().y()
                  << ", " << T_world_body.translation().z() << ")"
                  << " converged=" << match_result.is_converged
                  << " fitness=" << match_result.fitness_score << "\n";
    }

    // Linear alignment
    if (solveLinearAlignment()) {
        ready_ = true;
        std::clog << "[SfmInit] READY — g=(" << result_.gravity.x() << ", " << result_.gravity.y()
                  << ", " << result_.gravity.z() << ") |g|=" << result_.gravity.norm() << "\n";
        return true;
    }

    std::clog << "[SfmInit] Linear alignment failed\n";
    return false;
}

// ===========================================================================
// isReady / getResult
// ===========================================================================
bool SfmLioInitializer::isReady() const { return ready_; }

core::LioInitResult SfmLioInitializer::getResult() const { return result_; }

// ===========================================================================
// insertScanToMap — transform points to world and add to ikd-tree
// ===========================================================================
void SfmLioInitializer::insertScanToMap(const core::FeatureSet& features,
                                        const Eigen::Isometry3d& T_world_lidar) {
    const auto& cloud = features.raw_deskewed;

    // Build a dummy Keyframe to use addKeyFrame (transforms cloud_body by pose)
    auto kf = std::make_shared<core::Keyframe>(
        /*id=*/static_cast<uint64_t>(frames_.size()),
        /*timestamp=*/core::Timestamp{},
        /*pose=*/T_world_lidar,
        /*features=*/features,
        /*cloud_body=*/cloud,
        /*matched_result=*/core::ScanMatchResult{});

    map_builder_->addKeyFrame(kf);
}

// ===========================================================================
// preintegrateImu — simple Euler integration (no bias correction during init)
// ===========================================================================
SfmLioInitializer::PreintResult SfmLioInitializer::preintegrateImu(
    const std::vector<core::ImuData>& imu_batch, const Eigen::Vector3d& b_g) const {
    PreintResult result;
    if (imu_batch.size() < 2) return result;

    for (size_t i = 0; i + 1 < imu_batch.size(); ++i) {
        const double dt = core::getDeltaSec(imu_batch[i].timestamp, imu_batch[i + 1].timestamp);
        if (dt <= 0.0 || dt > 0.1) continue;  // skip bad intervals

        // Mid-point values with gyro bias correction
        const Eigen::Vector3d acc = 0.5 * (imu_batch[i].acc + imu_batch[i + 1].acc);
        const Eigen::Vector3d gyr = 0.5 * (imu_batch[i].gyr + imu_batch[i + 1].gyr) - b_g;

        // Integrate rotation (body frame)
        const Eigen::Matrix3d dR = expSO3(gyr * dt);

        // Integrate velocity and position in body frame of frame k
        result.alpha += result.beta * dt + 0.5 * result.delta_R * acc * dt * dt;
        result.beta += result.delta_R * acc * dt;
        result.delta_R = result.delta_R * dR;
        result.dt += dt;
    }
    return result;
}

// ===========================================================================
// estimateGyroBias — least-squares from rotation residuals
// ===========================================================================
//
// For each interval k:
//   dR_scan = R_k^T * R_{k+1}    (relative rotation from scan matching)
//   dR_imu  = preints[k].delta_R (preintegrated with b_g = 0)
//   residual: phi_k = Log(dR_scan^T * dR_imu)
//
// Linearising the rotation preintegration wrt b_g at the current b_g=0:
//   phi_k ≈ dt_k * b_g
//
// Stacking: A * b_g = rhs  →  A^T A * b_g = A^T rhs
// where A_k = dt_k * I,  rhs_k = phi_k
//
Eigen::Vector3d SfmLioInitializer::estimateGyroBias(
    const std::vector<PreintResult>& preints) const {
    const int N = static_cast<int>(frames_.size()) - 1;

    Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Atrhs = Eigen::Vector3d::Zero();

    for (int k = 0; k < N; ++k) {
        const double dt = preints[k].dt;
        if (dt < 1e-6) continue;

        const Eigen::Matrix3d dR_scan =
            frames_[k].pose.rotation().transpose() * frames_[k + 1].pose.rotation();
        const Eigen::Matrix3d dR_err = dR_scan.transpose() * preints[k].delta_R;

        // Log map: convert rotation matrix to axis-angle vector
        const Eigen::AngleAxisd aa(dR_err);
        const Eigen::Vector3d phi = aa.angle() * aa.axis();

        // Normal equations contribution
        AtA += (dt * dt) * Eigen::Matrix3d::Identity();
        Atrhs += dt * phi;
    }

    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
    if (AtA.determinant() > 1e-12) {
        b_g = AtA.ldlt().solve(Atrhs);
    }

    std::clog << "[SfmInit] estimateGyroBias: b_g=(" << b_g.x() << ", " << b_g.y() << ", "
              << b_g.z() << ") norm=" << b_g.norm() << " rad/s\n";
    return b_g;
}

// ===========================================================================
// solveLinearAlignment — VINS-Mono style linear system for {v_k, g}
// ===========================================================================
//
// For each pair of consecutive frames (k, k+1) with preintegrated IMU:
//
//   p_{k+1} = p_k + v_k * dt + 0.5 * g * dt^2 + R_k * alpha_k
//   v_{k+1} = v_k + g * dt + R_k * beta_k
//
// Rearranging into Ax = b, where x = [v_0, v_1, ..., v_N, g]^T ∈ R^{3(N+1)+3}:
//
//   Position eq:  I*dt * v_k - I * 0 * v_{k+1} + 0.5*dt^2 * g = p_{k+1} - p_k - R_k * alpha_k
//   Wait — v_{k+1} doesn't appear in position eq directly.
//
// Actually, the standard formulation uses two equations per interval:
//
//   alpha_k = R_k^T * (p_{k+1} - p_k - v_k*dt - 0.5*g*dt^2)
//   beta_k  = R_k^T * (v_{k+1} - v_k - g*dt)
//
// Rearranging:
//   R_k * alpha_k = p_{k+1} - p_k - v_k*dt - 0.5*g*dt^2
//   R_k * beta_k  = v_{k+1} - v_k - g*dt
//
// Position eq → -dt*I * v_k + 0*v_{k+1} - 0.5*dt^2*I * g = R_k*alpha_k - (p_{k+1} - p_k)
// Velocity eq → -I * v_k + I * v_{k+1} - dt*I * g = R_k*beta_k
//
// State vector x = [v_0(3), v_1(3), ..., v_N(3), g(3)]  → dim = 3*(N+1) + 3
// Per interval: 6 rows (3 position + 3 velocity), N intervals → 6*N total rows
//
bool SfmLioInitializer::solveLinearAlignment() {
    const int N = static_cast<int>(frames_.size()) - 1;  // number of intervals
    if (N < 2) return false;

    // -----------------------------------------------------------------------
    // Collect raw IMU batches for every inter-frame interval (reused across passes)
    // -----------------------------------------------------------------------
    std::vector<std::vector<core::ImuData>> batches(N);
    for (int k = 0; k < N; ++k) {
        const auto t_start = frames_[k].time;
        const auto t_end = frames_[k + 1].time;

        std::vector<core::ImuData>& batch = batches[k];

        const core::ImuData* before_start = nullptr;
        const core::ImuData* after_start = nullptr;
        const core::ImuData* before_end = nullptr;

        for (size_t j = 0; j < imu_buf_.size(); ++j) {
            if (imu_buf_[j].timestamp <= t_start) {
                before_start = &imu_buf_[j];
            } else if (!after_start) {
                after_start = &imu_buf_[j];
            }
            if (imu_buf_[j].timestamp < t_end) {
                before_end = &imu_buf_[j];
            }
        }

        if (before_start && after_start && after_start->timestamp > t_start) {
            batch.push_back(interpolateImu(*before_start, *after_start, t_start));
        }
        for (const auto& imu : imu_buf_) {
            if (imu.timestamp > t_start && imu.timestamp < t_end) {
                batch.push_back(imu);
            }
        }
        if (before_end && before_end->timestamp < t_end) {
            for (size_t j = 0; j < imu_buf_.size(); ++j) {
                if (imu_buf_[j].timestamp >= t_end) {
                    batch.push_back(interpolateImu(*before_end, imu_buf_[j], t_end));
                    break;
                }
            }
        }
    }

    // -----------------------------------------------------------------------
    // Pass 1: preintegrate with b_g = 0 to estimate gyro bias
    // -----------------------------------------------------------------------
    std::vector<PreintResult> preints(N);
    for (int k = 0; k < N; ++k) {
        preints[k] = preintegrateImu(batches[k]);
        if (preints[k].dt < 1e-6) {
            std::clog << "[SfmInit] Interval " << k << " has no valid IMU data\n";
            return false;
        }
    }

    // Estimate gyro bias from rotation residuals (scan-match vs IMU)
    const Eigen::Vector3d b_g_est = estimateGyroBias(preints);

    // -----------------------------------------------------------------------
    // Pass 2: re-preintegrate with estimated b_g correction
    // -----------------------------------------------------------------------
    for (int k = 0; k < N; ++k) {
        preints[k] = preintegrateImu(batches[k], b_g_est);
        if (preints[k].dt < 1e-6) return false;
    }

    // Build linear system: A * x = b
    // x = [v_0, v_1, ..., v_N, g] = 3*(N+1) + 3 unknowns
    const int state_dim = 3 * (N + 1) + 3;
    const int n_rows = 6 * N;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_rows, state_dim);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(n_rows);

    for (int k = 0; k < N; ++k) {
        const int row = 6 * k;
        const int v_k_col = 3 * k;
        const int v_k1_col = 3 * (k + 1);
        const int g_col = 3 * (N + 1);

        const Eigen::Matrix3d R_k = frames_[k].pose.rotation();
        const Eigen::Vector3d p_k = frames_[k].pose.translation();
        const Eigen::Vector3d p_k1 = frames_[k + 1].pose.translation();
        const double dt = preints[k].dt;
        const Eigen::Vector3d& alpha = preints[k].alpha;
        const Eigen::Vector3d& beta = preints[k].beta;

        // Position equation (3 rows):
        // -dt * v_k - 0.5*dt^2 * g = R_k * alpha - (p_{k+1} - p_k)
        A.block<3, 3>(row, v_k_col) = -dt * Eigen::Matrix3d::Identity();
        A.block<3, 3>(row, g_col) = -0.5 * dt * dt * Eigen::Matrix3d::Identity();
        b.segment<3>(row) = R_k * alpha - (p_k1 - p_k);

        // Velocity equation (3 rows):
        // -v_k + v_{k+1} - dt * g = R_k * beta
        A.block<3, 3>(row + 3, v_k_col) = -Eigen::Matrix3d::Identity();
        A.block<3, 3>(row + 3, v_k1_col) = Eigen::Matrix3d::Identity();
        A.block<3, 3>(row + 3, g_col) = -dt * Eigen::Matrix3d::Identity();
        b.segment<3>(row + 3) = R_k * beta;
    }

    // Solve via SVD (robust for potentially ill-conditioned systems)
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::VectorXd x = svd.solve(b);

    // Extract gravity
    const Eigen::Vector3d g_solved = x.segment<3>(3 * (N + 1));
    const double g_norm = g_solved.norm();

    std::clog << "[SfmInit] Linear alignment: g=(" << g_solved.x() << ", " << g_solved.y() << ", "
              << g_solved.z() << ") |g|=" << g_norm << "\n";

    // Sanity check gravity magnitude (should be ~9.8)
    if (g_norm < 8.0 || g_norm > 12.0) {
        std::clog << "[SfmInit] Gravity magnitude " << g_norm << " out of range, rejecting\n";
        return false;
    }

    // Extract velocities
    std::vector<Eigen::Vector3d> velocities(N + 1);
    for (int k = 0; k <= N; ++k) {
        velocities[k] = x.segment<3>(3 * k);
    }

    std::clog << "[SfmInit] v_0=(" << velocities[0].x() << ", " << velocities[0].y() << ", "
              << velocities[0].z() << ")\n";

    // --- Refine gravity to known magnitude ---
    // Project g_solved onto the sphere of radius |g_known| = 9.80511
    constexpr double kGravityMagnitude = 9.80511;
    Eigen::Vector3d g_refined = g_solved.normalized() * kGravityMagnitude;

    // --- Stage 2: 2-DOF manifold refinement (VINS-Mono §V-B) ---
    // Parametrise gravity on the sphere:  g = g0 + B * w,
    // where B = [b1, b2] is a 3×2 tangent-space basis at g0.
    g_refined = refineGravity(frames_, preints, g_refined, velocities);

    // --- Rotate world frame so that Z aligns with -gravity (gravity-aligned frame) ---
    // g_refined is currently in the init world frame (= first LiDAR body frame).
    // We want a new world frame where gravity = [0, 0, -G].
    // R_align rotates init_world → gravity_world.
    Eigen::Matrix3d R_gw = Eigen::Matrix3d::Identity();
    if (params_.align_gravity) {
        const Eigen::Quaterniond R_align = Eigen::Quaterniond::FromTwoVectors(
            g_refined, Eigen::Vector3d(0.0, 0.0, -kGravityMagnitude));
        R_gw = R_align.toRotationMatrix();
    } else {
        const double tilt_deg =
            std::acos(std::clamp(g_refined.normalized().dot(Eigen::Vector3d(0.0, 0.0, -1.0)), -1.0,
                                 1.0)) *
            180.0 / M_PI;
        std::clog << "[SfmInit] Gravity alignment disabled; keeping scan-matching world frame "
                  << "(solved tilt=" << tilt_deg << " deg)\n";
    }

    // --- Fill result (in gravity-aligned world frame) ---
    const int last = static_cast<int>(frames_.size()) - 1;
    result_.timestamp = frames_[last].time;
    result_.R = R_gw * frames_[last].pose.rotation();
    result_.p = R_gw * frames_[last].pose.translation();
    result_.v = R_gw * velocities[last];
    result_.b_a = Eigen::Vector3d::Zero();
    result_.b_g = b_g_est;
    result_.gravity = Eigen::Vector3d(0.0, 0.0, -kGravityMagnitude);

    return true;
}

Eigen::Vector3d SfmLioInitializer::refineGravity(
    const std::vector<SfmLioInitializer::ScanFrame>& frames,
    const std::vector<SfmLioInitializer::PreintResult>& preints, const Eigen::Vector3d& g_solved,
    std::vector<Eigen::Vector3d>& velocities) {
    const int N = static_cast<int>(frames_.size()) - 1;
    const int state_dim = 3 * (N + 1) + 3;
    const int n_rows = 6 * N;
    constexpr double kGravityMagnitude = 9.80511;
    Eigen::Vector3d g_refined = g_solved.normalized() * kGravityMagnitude;

    // --- Stage 2: 2-DOF manifold refinement (VINS-Mono §V-B) ---
    // Parametrise gravity on the sphere:  g = g0 + B * w,
    // where B = [b1, b2] is a 3×2 tangent-space basis at g0.
    constexpr int kRefineIters = 4;
    for (int refine = 0; refine < kRefineIters; ++refine) {
        // Tangent basis: pick the axis least aligned with g_refined, cross-product twice.
        const Eigen::Vector3d g_dir = g_refined.normalized();
        const Eigen::Vector3d ref =
            (std::abs(g_dir.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d b1 = (g_dir.cross(ref)).normalized();
        const Eigen::Vector3d b2 = g_dir.cross(b1);  // already unit since both inputs are unit

        // State: x = [v_0, ..., v_N, w1, w2]  →  3*(N+1) + 2
        const int state_dim2 = 3 * (N + 1) + 2;
        Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(n_rows, state_dim2);
        Eigen::VectorXd b2_vec = Eigen::VectorXd::Zero(n_rows);

        for (int k = 0; k < N; ++k) {
            const int row = 6 * k;
            const int v_k_col = 3 * k;
            const int v_k1_col = 3 * (k + 1);
            const int w_col = 3 * (N + 1);  // columns [w_col, w_col+1]

            const Eigen::Matrix3d R_k = frames_[k].pose.rotation();
            const Eigen::Vector3d p_k = frames_[k].pose.translation();
            const Eigen::Vector3d p_k1 = frames_[k + 1].pose.translation();
            const double dt_k = preints[k].dt;
            const Eigen::Vector3d& alpha_k = preints[k].alpha;
            const Eigen::Vector3d& beta_k = preints[k].beta;

            // Position eq: -dt*v_k - 0.5*dt^2 * B * w = R_k*alpha - (p_{k+1}-p_k) + 0.5*dt^2*g0
            A2.block<3, 3>(row, v_k_col) = -dt_k * Eigen::Matrix3d::Identity();
            A2.block<3, 1>(row, w_col) = -0.5 * dt_k * dt_k * b1;
            A2.block<3, 1>(row, w_col + 1) = -0.5 * dt_k * dt_k * b2;
            b2_vec.segment<3>(row) = R_k * alpha_k - (p_k1 - p_k) + 0.5 * dt_k * dt_k * g_refined;

            // Velocity eq: -v_k + v_{k+1} - dt * B * w = R_k*beta + dt*g0
            A2.block<3, 3>(row + 3, v_k_col) = -Eigen::Matrix3d::Identity();
            A2.block<3, 3>(row + 3, v_k1_col) = Eigen::Matrix3d::Identity();
            A2.block<3, 1>(row + 3, w_col) = -dt_k * b1;
            A2.block<3, 1>(row + 3, w_col + 1) = -dt_k * b2;
            b2_vec.segment<3>(row + 3) = R_k * beta_k + dt_k * g_refined;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd2(A2, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::VectorXd x2 = svd2.solve(b2_vec);
        const double w1 = x2(3 * (N + 1));
        const double w2 = x2(3 * (N + 1) + 1);

        // Update gravity and re-project to sphere
        g_refined = (g_refined + w1 * b1 + w2 * b2).normalized() * kGravityMagnitude;

        // Update velocities for the final iteration
        if (refine == kRefineIters - 1) {
            for (int k = 0; k <= N; ++k) {
                velocities[k] = x2.segment<3>(3 * k);
            }
        }

        std::clog << "[SfmInit] Refine iter " << refine << ": dw=(" << w1 << ", " << w2
                  << ") |g|=" << g_refined.norm() << "\n";
    }
    return g_refined;
}

// ===========================================================================
// clearScans — drop stale scans so the buffer can refill after a failed init
// ===========================================================================
void SfmLioInitializer::clearScans() {
    scan_buf_.clear();
    frames_.clear();
    // Reset the map so the next round starts from a blank slate
    map_builder_ = std::make_shared<map_builder::IkdTreeMapBuilder>(map_builder_params_);
    scan_matcher_ =
        std::make_shared<scan_matcher::IkdTreeScanMatcher>(map_builder_, scan_matcher_params_);
    scan_matcher_->setLidarExtrinsics(T_imu_lidar_);
}

}  // namespace lio_slam_shaw::initializer
