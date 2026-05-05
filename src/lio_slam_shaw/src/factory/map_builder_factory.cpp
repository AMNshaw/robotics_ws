#include "lio_slam_shaw/factory/map_builder_factory.hpp"

#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

namespace lio_slam_shaw::factory {

core::IMapBuilder::SharedPtr MapBuilderFactory::create(rclcpp::Node* node,
                                                       const Eigen::Isometry3d& T_base_lidar) {
    std::string type = node->declare_parameter<std::string>("map_builder.type", "ikd_tree");

    if (type == "ikd_tree") {
        map_builder::IkdTreeMapBuilderParams params;
        params.keyframe_distance_threshold =
            node->declare_parameter<double>("map_builder.keyframe_distance_threshold", 1.0);
        params.keyframe_angle_threshold =
            node->declare_parameter<double>("map_builder.keyframe_angle_threshold", 0.2);
        params.ikd_delete_param = static_cast<float>(
            node->declare_parameter<double>("map_builder.ikd_delete_param", 0.5));
        params.ikd_balance_param = static_cast<float>(
            node->declare_parameter<double>("map_builder.ikd_balance_param", 0.6));
        params.ikd_downsample_size = static_cast<float>(
            node->declare_parameter<double>("map_builder.ikd_downsample_size", 0.3));
        params.T_base_lidar = T_base_lidar;

        return std::make_shared<map_builder::IkdTreeMapBuilder>(params);
    }

    throw std::invalid_argument("Unknown map_builder type '" + type + "'.");
}

}  // namespace lio_slam_shaw::factory
