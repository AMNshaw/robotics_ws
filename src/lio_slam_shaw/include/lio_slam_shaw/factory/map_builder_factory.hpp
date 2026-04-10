#ifndef LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"

namespace lio_slam_shaw::factory {

class MapBuilderFactory {
public:
    // type 由 ROS param "map_builder_type" 控制
    // 支援: "ikd_tree" (預設)
    static core::IMapBuilder::SharedPtr create(rclcpp::Node::SharedPtr node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
