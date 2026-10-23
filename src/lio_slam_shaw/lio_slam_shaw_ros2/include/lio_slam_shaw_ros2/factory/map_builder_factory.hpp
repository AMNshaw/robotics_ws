#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_builder/i_map_builder_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosLocalMapBuilderFactory : public lio_slam_shaw::map_builder::ILocalMapBuilderFactory {
public:
    RosLocalMapBuilderFactory(rclcpp::Node* node);

    lio_slam_shaw::core::ILocalMapBuilder::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar,
        lio_slam_shaw::map_builder::LocalMapType type) const override;

private:
    rclcpp::Node* node_;
};

class RosGlobalMapBuilderFactory : public lio_slam_shaw::map_builder::IGlobalMapBuilderFactory {
public:
    RosGlobalMapBuilderFactory(rclcpp::Node* node);

    lio_slam_shaw::core::IGlobalMapBuilder::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar,
        lio_slam_shaw::map_builder::GlobalMapType type) const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_
