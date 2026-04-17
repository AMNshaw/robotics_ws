#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

#include <numeric>
#include <stdexcept>

namespace lio_slam_shaw::odometry_estimator {

FastLioOdometry::FastLioOdometry(core::IMapBuilder::SharedPtr map_builder,
                                 const FastLioOdometryParams& params)
    : params_(params) {
    map_builder_ = std::dynamic_pointer_cast<map_builder::IkdTreeMapBuilder>(map_builder);
    if (!map_builder_) {
        throw std::invalid_argument("FastLioOdometry requires an IkdTreeMapBuilder instance");
    }

    // Build process noise matrix Q (diagonal, continuous-time noise → discrete handled in
    // propagate)
    Q_.setZero();
    // [0:3]   rotation noise from gyro measurement noise
    Q_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * params_.gyr_noise * params_.gyr_noise;
    // [3:6]   position noise (negligible direct noise, driven by velocity)
    // [6:9]   velocity noise from accel measurement noise
    Q_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * params_.acc_noise * params_.acc_noise;
    // [9:12]  gyro bias random walk
    Q_.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * params_.gyr_bias_noise * params_.gyr_bias_noise;
    // [12:15] accel bias random walk
    Q_.block<3, 3>(12, 12) =
        Eigen::Matrix3d::Identity() * params_.acc_bias_noise * params_.acc_bias_noise;
    // [15:18] gravity (treated as constant, no noise)

    prev_scan_time_ = core::Timestamp::min();
}

// ---------------------------------------------------------------------------
// IOdometryEstimator interface
// ---------------------------------------------------------------------------

void FastLioOdometry::feedImu(const core::ImuData& imu) {
    if (!bias_initialized_) {
        tryInitBias(imu);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    imu_buf_.push_back(imu);

    // Trim stale IMU data (keep at most 10 seconds behind prev_scan_time_)
    constexpr auto kMaxLag = std::chrono::seconds(10);
    while (imu_buf_.size() > 1 &&
           (imu_buf_.front().timestamp + kMaxLag) < imu_buf_.back().timestamp) {
        imu_buf_.pop_front();
    }
}

core::OdometryResult FastLioOdometry::estimateWithFeatures(const core::FeatureSet& features,
                                                           core::Timestamp lidar_time_start) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    // 1. Collect IMU batch from prev scan to this scan start
    std::vector<core::ImuData> imu_batch;
    for (const auto& imu : imu_buf_) {
        if (prev_scan_time_ == core::Timestamp::min() || imu.timestamp > prev_scan_time_) {
            if (imu.timestamp <= lidar_time_start) {
                imu_batch.push_back(imu);
            }
        }
    }

    // 2. Forward propagation (IMU integration + covariance propagation)
    if (!imu_batch.empty()) {
        double dt_total = core::getDeltaSec(prev_scan_time_ == core::Timestamp::min()
                                                ? imu_batch.front().timestamp
                                                : prev_scan_time_,
                                            lidar_time_start);
        propagate(imu_batch, dt_total);
    }

    // 3. iEKF iterated update with point-to-plane residuals
    IeskfState corrected = iteratedUpdate(features, state_);
    state_ = corrected;

    prev_scan_time_ = lidar_time_start;

    // 4. Build result in odom frame (T_map_odom_ = Identity until loop closure)
    Eigen::Isometry3d pose_map = Eigen::Isometry3d::Identity();
    pose_map.linear() = state_.R;
    pose_map.translation() = state_.p;

    core::ScanMatchResult matched_in_map;
    matched_in_map.pose = pose_map;
    matched_in_map.is_converged = true;

    core::ScanMatchResult matched_in_odom;
    matched_in_odom.pose = T_map_odom_.inverse() * pose_map;
    matched_in_odom.is_converged = true;

    core::NavState nav;
    nav.timestamp = lidar_time_start;
    nav.pose = matched_in_odom.pose;
    nav.linear_vel = state_.v;
    nav.acc_bias = state_.b_a;
    nav.gyr_bias = state_.b_g;

    return core::OdometryResult{matched_in_map, matched_in_odom, nav};
}

core::NavState FastLioOdometry::getLatestState() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    core::NavState nav;
    nav.pose.linear() = state_.R;
    nav.pose.translation() = state_.p;
    nav.linear_vel = state_.v;
    nav.acc_bias = state_.b_a;
    nav.gyr_bias = state_.b_g;
    return nav;
}

void FastLioOdometry::setMapToOdomTransform(const Eigen::Isometry3d& T_map_odom) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    T_map_odom_ = T_map_odom;
}

void FastLioOdometry::setLidarExtrinsics(const Eigen::Isometry3d& T_base_lidar) {
    T_base_lidar_ = T_base_lidar;
    // T_imu_lidar_ is updated when setImuExtrinsics is also called
}

void FastLioOdometry::setImuExtrinsics(const Eigen::Isometry3d& T_base_imu) {
    T_imu_lidar_ = T_base_imu.inverse() * T_base_lidar_;
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

void FastLioOdometry::propagate(const std::vector<core::ImuData>& imu_batch, double /*dt_total*/) {
    // TODO: implement continuous-time IMU integration on SO(3) manifold
    // For each consecutive IMU pair (or single measurement):
    //   dt = t_{k+1} - t_k
    //   omega = imu.gyr - state_.b_g
    //   a     = imu.acc - state_.b_a
    //   state_.R = state_.R * Exp(omega * dt)
    //   state_.v += (state_.R * a + gravity_) * dt
    //   state_.p += state_.v * dt + 0.5*(state_.R*a + gravity_)*dt*dt
    //   Propagate P via Jacobian F and noise Q:
    //   P = F * P * F^T + G * Q * G^T * dt
    (void)imu_batch;
}

IeskfState FastLioOdometry::iteratedUpdate(const core::FeatureSet& features,
                                           const IeskfState& propagated) {
    // TODO: implement iEKF iterated update
    // For iter = 0..max_iterations:
    //   For each point in features.raw_deskewed:
    //     buildPointResidual(pt, state_iter, H_i, r_i)
    //   Assemble H (N×18), r (N×1)
    //   K = P * H^T * (H * P * H^T + R_meas)^-1
    //   dx = K * r
    //   state_iter = state_iter ⊞ dx   (manifold boxplus)
    //   if ||dx|| < threshold: break
    // P = (I - K*H) * P
    (void)features;
    return propagated;
}

bool FastLioOdometry::buildPointResidual(const core::PointXYZIRT& pt_body, const IeskfState& state,
                                         Eigen::Matrix<double, 1, 18>& H, double& r) const {
    // TODO: transform pt_body → world frame, search 5-NN in ikd-tree,
    //       fit plane via SVD, compute point-to-plane distance as residual,
    //       compute Jacobian w.r.t. error-state.
    (void)pt_body;
    (void)state;
    H.setZero();
    r = 0.0;
    return false;
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

    // Gyro bias = mean gyro reading (robot is static)
    state_.b_g = mean_gyr;

    // Accel bias: mean_acc should equal -R^T * g when static.
    // Assume initial pose is level → gravity along -Z in body frame.
    state_.b_a = mean_acc - Eigen::Vector3d(0.0, 0.0, kGravity);

    bias_initialized_ = true;
    static_imu_buf_.clear();
    static_imu_buf_.shrink_to_fit();
}

}  // namespace lio_slam_shaw::odometry_estimator
