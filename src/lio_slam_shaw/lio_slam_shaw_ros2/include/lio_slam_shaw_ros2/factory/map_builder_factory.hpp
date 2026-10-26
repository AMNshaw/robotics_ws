#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_builder/i_map_builder_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace map_builder = lio_slam_shaw::map_builder;

class RosLocalMapBuilderFactory : public map_builder::ILocalMapBuilderFactory {
public:
    RosLocalMapBuilderFactory(rclcpp::Node* node) : node_(node) {}

    core::ILocalMapBuilder::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                             map_builder::LocalMapType type) const override;

private:
    rclcpp::Node* node_;
};

class RosGlobalMapBuilderFactory : public map_builder::IGlobalMapBuilderFactory {
public:
    RosGlobalMapBuilderFactory(rclcpp::Node* node) : node_(node) {}

    core::IGlobalMapBuilder::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                              map_builder::GlobalMapType type) const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__MAP_BUILDER_FACTORY_HPP_
