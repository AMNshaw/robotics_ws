#include "lio_slam_shaw/factory/map_builder_factory.hpp"

#include "lio_slam_shaw/map_builder/ikd_tree_global_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::factory {

std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> LocalMapBuilderFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar) {
    map_builder::IkdTreeLocalMapBuilderParams params;
    params.ikd_delete_param =
        static_cast<float>(node->declare_parameter<double>("local_map.ikd_delete_param", 0.5));
    params.ikd_balance_param =
        static_cast<float>(node->declare_parameter<double>("local_map.ikd_balance_param", 0.6));
    params.ikd_downsample_size =
        static_cast<float>(node->declare_parameter<double>("local_map.ikd_downsample_size", 0.3));
    params.box_trim_half_length =
        node->declare_parameter<double>("local_map.box_trim_half_length", 50.0);
    params.T_base_lidar = T_base_lidar;

    return std::make_shared<map_builder::IkdTreeLocalMapBuilder>(params);
}

core::IGlobalMapBuilder::SharedPtr GlobalMapBuilderFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar) {
    map_builder::IkdTreeGlobalMapBuilderParams params;
    params.keyframe_distance_threshold =
        node->declare_parameter<double>("global_map.keyframe_distance_threshold", 1.0);
    params.keyframe_angle_threshold =
        node->declare_parameter<double>("global_map.keyframe_angle_threshold", 0.2);
    params.T_base_lidar = T_base_lidar;

    return std::make_shared<map_builder::IkdTreeGlobalMapBuilder>(params);
}

}  // namespace lio_slam_shaw::factory
