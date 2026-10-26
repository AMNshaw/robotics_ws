#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__SLAM_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__SLAM_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/slam_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosSlamFactory : public lio_slam_shaw::core::ISlamFactory {
public:
    RosSlamFactory(rclcpp::Node* node);

private:
    lio_slam_shaw::core::FrontEnd::SharedPtr createFrontEnd(
        const lio_slam_shaw::core::Extrinsics& extrinsics) const override;
    lio_slam_shaw::core::BackEnd::SharedPtr createBackEnd(
        const lio_slam_shaw::core::Extrinsics& extrinsics) const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__SLAM_FACTORY_HPP_