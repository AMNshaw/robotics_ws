#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/odometry_estimator/i_odometry_estimator_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

enum class OdometryEstimatorType { FAST_LIO };

class RosOdometryEstimatorFactory
    : public lio_slam_shaw::odometry_estimator::IOdometryEstimatorFactory {
public:
    RosOdometryEstimatorFactory(rclcpp::Node* node);

    lio_slam_shaw::core::IOdometryEstimator::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        lio_slam_shaw::core::ILocalMapBuilder::SharedPtr local_map,
        lio_slam_shaw::odometry_estimator::OdometryEstimatorType type) const override;

private:
    lio_slam_shaw::core::IOdometryEstimator::SharedPtr createFastLio(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu,
        std::shared_ptr<lio_slam_shaw::map_builder::IkdTreeLocalMapBuilder> local_map)
        const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__ODOMETRY_ESTIMATOR_FACTORY_HPP_
