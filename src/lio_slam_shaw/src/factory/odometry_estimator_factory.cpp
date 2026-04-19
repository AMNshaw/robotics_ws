#include "lio_slam_shaw/factory/odometry_estimator_factory.hpp"

#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

namespace lio_slam_shaw::factory {

core::IOdometryEstimator::SharedPtr OdometryEstimatorFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
    core::IMapBuilder::SharedPtr map_builder) {
    std::string type = node->declare_parameter<std::string>("odometry_estimator.type", "fast_lio");

    if (type == "fast_lio") {
        return createFastLio(node, T_base_lidar, T_base_imu, map_builder);
    }

    RCLCPP_WARN(node->get_logger(),
                "Unknown odometry_estimator type '%s', falling back to fast_lio.", type.c_str());
    return createFastLio(node, T_base_lidar, T_base_imu, map_builder);
}

core::IOdometryEstimator::SharedPtr OdometryEstimatorFactory::createFastLio(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
    core::IMapBuilder::SharedPtr map_builder) {
    odometry_estimator::FastLioOdometryParams params;

    params.acc_noise =
        node->declare_parameter<double>("odometry_estimator.acc_noise", params.acc_noise);
    params.gyr_noise =
        node->declare_parameter<double>("odometry_estimator.gyr_noise", params.gyr_noise);
    params.acc_bias_noise =
        node->declare_parameter<double>("odometry_estimator.acc_bias_noise", params.acc_bias_noise);
    params.gyr_bias_noise =
        node->declare_parameter<double>("odometry_estimator.gyr_bias_noise", params.gyr_bias_noise);
    params.measurement_noise = node->declare_parameter<double>(
        "odometry_estimator.measurement_noise", params.measurement_noise);

    params.max_iterations =
        node->declare_parameter<int>("odometry_estimator.max_iterations", params.max_iterations);
    params.state_converge_threshold = node->declare_parameter<double>(
        "odometry_estimator.state_converge_threshold", params.state_converge_threshold);

    params.num_nearest_neighbors = node->declare_parameter<int>(
        "odometry_estimator.num_nearest_neighbors", params.num_nearest_neighbors);
    params.min_plane_points = node->declare_parameter<int>("odometry_estimator.min_plane_points",
                                                           params.min_plane_points);
    params.search_radius = static_cast<float>(
        node->declare_parameter<double>("odometry_estimator.search_radius", params.search_radius));
    params.min_plane_eigenvalue_ratio = node->declare_parameter<double>(
        "odometry_estimator.min_plane_eigenvalue_ratio", params.min_plane_eigenvalue_ratio);
    params.max_point_to_plane_distance = node->declare_parameter<double>(
        "odometry_estimator.max_point_to_plane_distance", params.max_point_to_plane_distance);

    auto estimator = std::make_shared<odometry_estimator::FastLioOdometry>(
        map_builder, T_base_lidar, T_base_imu, params);

    RCLCPP_INFO(node->get_logger(), "Created FastLioOdometry estimator");
    return estimator;
}

}  // namespace lio_slam_shaw::factory
