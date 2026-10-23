#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__LIO_INITIALIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__LIO_INITIALIZER_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/initializer/i_lio_initializer_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosLioInitializerFactory : public lio_slam_shaw::initializer::ILioInitializerFactory {
public:
    RosLioInitializerFactory(rclcpp::Node* node);

    lio_slam_shaw::core::ILioInitializer::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar, const Eigen::Isometry3d& T_base_imu) const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__LIO_INITIALIZER_FACTORY_HPP_
