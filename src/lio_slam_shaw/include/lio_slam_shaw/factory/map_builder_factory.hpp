#ifndef LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_global_map_builder.hpp"
#include "lio_slam_shaw/core/i_local_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::factory {

enum class LocalMapType { IKD_TREE };
enum class GlobalMapType { IKD_TREE };

class LocalMapBuilderFactory {
public:
    static std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> create(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar, LocalMapType type);
};

class GlobalMapBuilderFactory {
public:
    static core::IGlobalMapBuilder::SharedPtr create(rclcpp::Node* node,
                                                     const Eigen::Isometry3d& T_base_lidar,
                                                     GlobalMapType type);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__MAP_BUILDER_FACTORY_HPP_
