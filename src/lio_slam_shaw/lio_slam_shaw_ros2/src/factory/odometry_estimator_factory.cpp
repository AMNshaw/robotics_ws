#include "lio_slam_shaw_ros2/factory/odometry_estimator_factory.hpp"

#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

namespace lio_slam_shaw_ros2::factory {

core::IOdometryEstimator::SharedPtr RosOdometryEstimatorFactory::create(
    const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
    core::ILocalMapBuilder::SharedPtr local_map,
    odometry_estimator::OdometryEstimatorType type) const {
    switch (type) {
        case odometry_estimator::OdometryEstimatorType::FAST_LIO:
            return createFastLio(T_base_lidar, T_base_imu, local_map);
    }
    throw std::invalid_argument("OdometryEstimatorFactory: unknown type");
}

core::IOdometryEstimator::SharedPtr RosOdometryEstimatorFactory::createFastLio(
    const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
    core::ILocalMapBuilder::SharedPtr local_map) const {
    odometry_estimator::FastLioOdometryParams params;

    params.acc_noise =
        node_->declare_parameter<double>("frontend.odometry_estimator.acc_noise", params.acc_noise);
    params.gyr_noise =
        node_->declare_parameter<double>("frontend.odometry_estimator.gyr_noise", params.gyr_noise);
    params.acc_bias_noise = node_->declare_parameter<double>(
        "frontend.odometry_estimator.acc_bias_noise", params.acc_bias_noise);
    params.gyr_bias_noise = node_->declare_parameter<double>(
        "frontend.odometry_estimator.gyr_bias_noise", params.gyr_bias_noise);
    params.measurement_noise = node_->declare_parameter<double>(
        "frontend.odometry_estimator.measurement_noise", params.measurement_noise);

    params.max_iterations = node_->declare_parameter<int>(
        "frontend.odometry_estimator.max_iterations", params.max_iterations);
    params.state_converge_threshold = node_->declare_parameter<double>(
        "frontend.odometry_estimator.state_converge_threshold", params.state_converge_threshold);
    params.estimate_gravity = node_->declare_parameter<bool>(
        "frontend.odometry_estimator.estimate_gravity", params.estimate_gravity);

    params.num_nearest_neighbors = node_->declare_parameter<int>(
        "frontend.odometry_estimator.num_nearest_neighbors", params.num_nearest_neighbors);
    params.min_plane_points = node_->declare_parameter<int>(
        "frontend.odometry_estimator.min_plane_points", params.min_plane_points);
    params.search_radius = static_cast<float>(node_->declare_parameter<double>(
        "frontend.odometry_estimator.search_radius", params.search_radius));
    params.min_plane_eigenvalue_ratio =
        node_->declare_parameter<double>("frontend.odometry_estimator.min_plane_eigenvalue_ratio",
                                         params.min_plane_eigenvalue_ratio);
    params.max_point_to_plane_distance =
        node_->declare_parameter<double>("frontend.odometry_estimator.max_point_to_plane_distance",
                                         params.max_point_to_plane_distance);

    auto estimator = std::make_shared<odometry_estimator::FastLioOdometry>(local_map, T_base_lidar,
                                                                           T_base_imu, params);

    RCLCPP_INFO(node_->get_logger(), "Created FastLioOdometry estimator");
    return estimator;
}

}  // namespace lio_slam_shaw_ros2::factory
