#ifndef LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"

namespace lio_slam_shaw::factory {

class MapBuilderFactory {
public:
    static core::IMapBuilder::SharedPtr create(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar = Eigen::Isometry3d::Identity());
};
}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
