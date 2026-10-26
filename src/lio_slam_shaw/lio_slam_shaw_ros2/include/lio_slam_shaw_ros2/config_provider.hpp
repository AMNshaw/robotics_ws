#ifndef LIO_SLAM_SHAW_ROS2__CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW_ROS2__CONFIG_PROVIDER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_config_provider.hpp"

namespace lio_slam_ros2 {

template <typename TConfig>
class RosConfigProvider : lio_slam_shaw::core::IConfigProvider<TConfig> {
public:
    RosConfigProvider(rclcpp::Node* node) : node_(node) {}

    TConfig load() const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_ros2

#endif  // LIO_SLAM_SHAW_ROS2__CONFIG_PROVIDER_HPP_