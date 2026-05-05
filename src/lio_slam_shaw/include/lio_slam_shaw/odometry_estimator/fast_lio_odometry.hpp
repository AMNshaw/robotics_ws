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
    double measurement_noise = 0.01;  // point-to-plane residual noise (for covariance computation)

    // iEKF solver
    int max_iterations = 5;
    double state_converge_threshold = 1e-3;  // ||dx|| < threshold → converge

    // Point-to-plane matching (reuses the same scan search logic)
    int num_nearest_neighbors = 5;
    int min_plane_points = 3;
    double plane_valid_threshold = 0.1;  // max eigenvalue ratio for valid
    float search_radius = 1.0f;
    double min_plane_eigenvalue_ratio = 0.1;   // planarity check
    double max_point_to_plane_distance = 0.5;  // outlier rejection threshold
};

// iEKF state vector (17-DOF on manifold):
//   rotation    R ∈ SO(3)   — world←body
//   position    p ∈ R^3     — in world frame
//   velocity    v ∈ R^3     — in world frame
//   accel bias  b_a ∈ R^3
//   gyro  bias  b_g ∈ R^3
//   gravity dir δg ∈ R^2    — tangent-space perturbation of g on S²(G)
//
// Error-state ordering:
//   [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:17)]

static constexpr int kStateDim = 17;

class FastLioOdometry : public core::IOdometryEstimator {
public:
    using SharedPtr = std::shared_ptr<FastLioOdometry>;

    FastLioOdometry(core::IMapBuilder::SharedPtr map_builder, const Eigen::Isometry3d& T_base_lidar,
                    const Eigen::Isometry3d& T_base_imu, const FastLioOdometryParams& params = {});
    ~FastLioOdometry() override = default;

    // IOdometryEstimator interface
    void feedImu(const core::ImuData& imu) override;
    core::OdometryResult estimateWithFeatures(const core::FeatureSet& features,
                                              core::Timestamp lidar_time_start) override;
    core::NavState getLatestState() const override;
    std::vector<core::NavState> getNavStateQueueSnapshot() const override;
    void setInitialState(const core::LioInitResult& init_result) override;
    void setMapToOdomTransform(const Eigen::Isometry3d& T_map_odom) override;

private:
    struct IeskfState {
        core::Timestamp timestamp;
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        Eigen::Vector3d v = Eigen::Vector3d::Zero();
        Eigen::Vector3d b_g = Eigen::Vector3d::Zero();
        Eigen::Vector3d b_a = Eigen::Vector3d::Zero();
        Eigen::Vector3d gravity{0.0, 0.0, -9.80511};  // estimated gravity in world frame
        // Bias-corrected angular velocity from the last IMU measurement (for nav output)
        Eigen::Vector3d angular_vel = Eigen::Vector3d::Zero();

        // Tangent basis [b1, b2] at current gravity (columns of 3×2 matrix).
        // Updated whenever gravity changes.
        Eigen::Matrix<double, 3, 2> gravity_basis = Eigen::Matrix<double, 3, 2>::Zero();

        void updateGravityBasis() {
            const Eigen::Vector3d g_dir = gravity.normalized();
            const Eigen::Vector3d ref =
                (std::abs(g_dir.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
            gravity_basis.col(0) = (g_dir.cross(ref)).normalized();
            gravity_basis.col(1) = g_dir.cross(gravity_basis.col(0));
        }

        // Error-state covariance (17×17)
        // [δp(0:3), δv(3:6), δθ(6:9), δb_a(9:12), δb_g(12:15), δg(15:17)]
        Eigen::Matrix<double, kStateDim, kStateDim> P = []() {
            Eigen::Matrix<double, kStateDim, kStateDim> P0 =
                Eigen::Matrix<double, kStateDim, kStateDim>::Zero();
            P0.diagonal().segment<3>(0).setConstant(1e-3);   // position
            P0.diagonal().segment<3>(3).setConstant(1e-3);   // velocity
            P0.diagonal().segment<3>(6).setConstant(1e-3);   // rotation
            P0.diagonal().segment<3>(9).setConstant(1e-5);   // acc bias
            P0.diagonal().segment<3>(12).setConstant(1e-4);  // gyr bias
            P0.diagonal().segment<2>(15).setConstant(1e-2);  // gravity direction
            return P0;
        }();
    };

    struct NearestPlaneResult {
        bool valid = false;
        Eigen::Vector3d point_in_map;
        Eigen::Vector3d normal;
        Eigen::Vector3d centroid;
    };

    struct PointResidual {
        bool valid = false;
        Eigen::Matrix<double, 1, kStateDim> H = Eigen::Matrix<double, 1, kStateDim>::Zero();
        double r = 0.0;
    };

    // IMU forward propagation: integrate imu_buf_[prev_scan_time .. lidar_time_start]
    // Updates state_ in-place and accumulates process noise into state_.P.
    // IMU forward propagation: integrate imu_batch into the given state in-place,
    // updating both nominal state and error covariance P.
    IeskfState propagateStep(const IeskfState& state, const core::ImuData& imu);

    IeskfState predictStep(const IeskfState& state, const core::ImuData& imu);

    // iEKF measurement update using point-to-plane residuals.
    // Returns the converged state (does not modify state_ until accepted).
    IeskfState iteratedUpdate(const IeskfState& propagated,
                              const core::FeatureSet& features);  // NOLINT
    NearestPlaneResult fitPlane(const std::vector<lio_slam_shaw::core::PointXYZIRT>& neighbors,
                                const Eigen::Vector3d& query_point_in_map) const;

    NearestPlaneResult fitPlaneDirect(const map_builder::IkdTreeMapBuilder::PointVector& neighbors,
                                      const Eigen::Vector3d& query_point_in_map) const;

    // Transform pt_lidar to the map frame using T_world_lidar, search K nearest neighbours in the
    // ikd-tree, and fit a plane.  Thread-safe (uses thread_local buffers internally).
    NearestPlaneResult queryNearestPlane(const core::PointXYZIRT& pt_lidar,
                                         const Eigen::Isometry3d& T_world_lidar) const;

    // Compute point-to-plane Jacobian and residual for one point.
    // plane must already be valid; sensor_origin_in_map is used only for normal flipping.
    PointResidual buildPointResidual(const NearestPlaneResult& plane,
                                     const Eigen::Vector3d& p_lidar,
                                     const Eigen::Vector3d& sensor_origin_in_map,
                                     const Eigen::Matrix3d& R_map_body) const;

    FastLioOdometryParams params_;
    std::shared_ptr<map_builder::IkdTreeMapBuilder> map_builder_;

    Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_base_lidar_ = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_imu_lidar_ =
        Eigen::Isometry3d::Identity();  // cached T_base_imu^-1 * T_base_lidar

    mutable std::mutex predicted_states_mutex_;
    std::deque<IeskfState> predicted_states_;
    mutable std::mutex committed_state_mutex_;
    IeskfState committed_state_;  // last accepted state after iteratedUpdate converges

    // Raw IMU ring buffer; trimmed to [prev_scan_time, latest] on each scan
    mutable std::mutex imu_buf_mutex_;
    std::deque<core::ImuData> imu_buf_;
    core::Timestamp prev_scan_time_;

    // Gravity vector in world frame — initial value only; per-state gravity overrides this.
    static constexpr double kGravity = 9.80511;
    Eigen::Vector3d gravity_{0.0, 0.0, -kGravity};
};

}  // namespace lio_slam_shaw::odometry_estimator

#endif  // LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__FAST_LIO_ODOMETRY_HPP_
