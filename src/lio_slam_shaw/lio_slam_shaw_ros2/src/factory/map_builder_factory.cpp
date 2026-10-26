#include "lio_slam_shaw_ros2/factory/map_builder_factory.hpp"

#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/map_builder/keyframe_global_map_builder.hpp"

namespace lio_slam_shaw_ros2::factory {

core::ILocalMapBuilder::SharedPtr RosLocalMapBuilderFactory::create(
    const Eigen::Isometry3d& T_base_lidar, map_builder::LocalMapType type) const {
    map_builder::IkdTreeLocalMapBuilderParams params;
    params.ikd_delete_param = static_cast<float>(
        node_->declare_parameter<double>("frontend.local_map.ikd_delete_param", 0.5));
    params.ikd_balance_param = static_cast<float>(
        node_->declare_parameter<double>("frontend.local_map.ikd_balance_param", 0.6));
    params.ikd_downsample_size = static_cast<float>(
        node_->declare_parameter<double>("frontend.local_map.ikd_downsample_size", 0.3));
    params.box_trim_half_length =
        node_->declare_parameter<double>("frontend.local_map.box_trim_half_length", 50.0);
    params.T_base_lidar = T_base_lidar;

    return std::make_shared<map_builder::IkdTreeLocalMapBuilder>(params);
}

core::IGlobalMapBuilder::SharedPtr RosGlobalMapBuilderFactory::create(
    const Eigen::Isometry3d& T_base_lidar, map_builder::GlobalMapType type) const {
    map_builder::KeyframeGlobalMapBuilderParams params;
    params.keyframe_distance_threshold =
        node_->declare_parameter<double>("backend.global_map.keyframe_distance_threshold", 1.0);
    params.keyframe_angle_threshold =
        node_->declare_parameter<double>("backend.global_map.keyframe_angle_threshold", 0.2);
    params.T_base_lidar = T_base_lidar;

    return std::make_shared<map_builder::KeyframeGlobalMapBuilder>(params);
}

}  // namespace lio_slam_shaw_ros2::factory
