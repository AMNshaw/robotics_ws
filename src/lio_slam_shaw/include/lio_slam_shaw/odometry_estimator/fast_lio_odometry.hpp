#ifndef LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__FAST_LIO_ODOMETRY_HPP_
#define LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__FAST_LIO_ODOMETRY_HPP_

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <mutex>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_odometry_estimator.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

namespace lio_slam_shaw::odometry_estimator {

struct FastLioOdometryParams {
    // IMU noise (continuous-time, same convention as gtsam_imu_preintegrator)
    double acc_noise = 3.9939570888117902e-03;
    double gyr_noise = 1.5636343949698187e-03;
    double acc_bias_noise = 6.4356659353532566e-05;
    double gyr_bias_noise = 3.5640318696367613e-05;

    // iEKF solver
    int max_iterations = 30;
    double state_converge_threshold = 1e-6;  // ||dx|| < threshold → converge

    // Point-to-plane matching (reuses the same scan search logic)
    int num_nearest_neighbors = 5;
    float search_radius = 1.0f;
    double min_plane_eigenvalue_ratio = 0.1;   // planarity check
    double max_point_to_plane_distance = 0.3;  // outlier rejection threshold
};

// iEKF state vector (18-DOF on manifold):
//   rotation    R ∈ SO(3)   — world←body
//   position    p ∈ R^3     — in world frame
//   velocity    v ∈ R^3     — in world frame
//   gyro  bias  b_g ∈ R^3
//   accel bias  b_a ∈ R^3
struct IeskfState {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();

    // Error-state covariance (18×18)
    Eigen::Matrix<double, 18, 18> P = Eigen::Matrix<double, 18, 18>::Identity() * 1e-4;
};

class FastLioOdometry : public core::IOdometryEstimator {
public:
    using SharedPtr = std::shared_ptr<FastLioOdometry>;

    FastLioOdometry(core::IMapBuilder::SharedPtr map_builder,
                    const FastLioOdometryParams& params = {});
    ~FastLioOdometry() override = default;

    // IOdometryEstimator interface
    void feedImu(const core::ImuData& imu) override;
    core::OdometryResult estimateWithFeatures(const core::FeatureSet& features,
                                              core::Timestamp lidar_time_start) override;
    core::NavState getLatestState() const override;
    void setMapToOdomTransform(const Eigen::Isometry3d& T_map_odom) override;
    void setLidarExtrinsics(const Eigen::Isometry3d& T_base_lidar) override;
    void setImuExtrinsics(const Eigen::Isometry3d& T_base_imu) override;

private:
    // IMU forward propagation: integrate imu_buf_[prev_scan_time .. lidar_time_start]
    // Updates state_ in-place and accumulates process noise into state_.P.
    void propagate(const std::vector<core::ImuData>& imu_batch, double dt_total);

    // iEKF measurement update using point-to-plane residuals.
    // Returns the converged state (does not modify state_ until accepted).
    IeskfState iteratedUpdate(const core::FeatureSet& features,
                              const IeskfState& propagated);  // NOLINT

    // Build point-to-plane Jacobian H (1×18) and residual r for one point.
    // Returns false if the point is degenerate (no valid plane found).
    bool buildPointResidual(const core::PointXYZIRT& pt_body, const IeskfState& state,
                            Eigen::Matrix<double, 1, 18>& H, double& r) const;

    // Static bias initialisation from the first N static IMU samples.
    void tryInitBias(const core::ImuData& imu);

    FastLioOdometryParams params_;
    std::shared_ptr<map_builder::IkdTreeMapBuilder> map_builder_;

    Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_imu_lidar_ =
        Eigen::Isometry3d::Identity();  // cached T_base_imu^-1 * T_base_lidar

    mutable std::mutex state_mutex_;
    IeskfState state_;

    // Raw IMU ring buffer; trimmed to [prev_scan_time, latest] on each scan
    std::deque<core::ImuData> imu_buf_;
    core::Timestamp prev_scan_time_;
    bool is_initialized_ = false;

    // Static IMU bias init (same approach as GtsamImuPreintegrator)
    static constexpr int kMinStaticSamples = 200;
    static constexpr int kMaxStaticSamples = 500;
    std::vector<core::ImuData> static_imu_buf_;
    bool bias_initialized_ = false;

    // Process noise matrix Q (18×18, built once in constructor)
    Eigen::Matrix<double, 18, 18> Q_;

    // Gravity vector in world frame
    static constexpr double kGravity = 9.80511;
    Eigen::Vector3d gravity_{0.0, 0.0, -kGravity};
};

}  // namespace lio_slam_shaw::odometry_estimator

#endif  // LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__FAST_LIO_ODOMETRY_HPP_
