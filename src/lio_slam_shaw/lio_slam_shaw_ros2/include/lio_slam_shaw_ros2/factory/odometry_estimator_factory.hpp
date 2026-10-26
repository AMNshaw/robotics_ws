#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/odometry_estimator/i_odometry_estimator_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace odometry_estimator = lio_slam_shaw::odometry_estimator;
namespace map_builder = lio_slam_shaw::map_builder;

class RosOdometryEstimatorFactory
    : public lio_slam_shaw::odometry_estimator::IOdometryEstimatorFactory {
public:
    RosOdometryEstimatorFactory(rclcpp::Node* node) : node_(node) {}

    core::IOdometryEstimator::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        core::ILocalMapBuilder::SharedPtr local_map,
        odometry_estimator::OdometryEstimatorType type) const override;

private:
    core::IOdometryEstimator::SharedPtr createFastLio(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        core::ILocalMapBuilder::SharedPtr local_map) const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
