#include "lio_slam_shaw/factory/map_builder_factory.hpp"

#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

namespace lio_slam_shaw::factory {

core::IMapBuilder::SharedPtr MapBuilderFactory::create(rclcpp::Node* node) {
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
        params.max_search_dist =
            node->declare_parameter<double>("map_builder.max_search_dist", 2.0);
        params.min_plane_points = node->declare_parameter<int>("map_builder.min_plane_points", 5);
        params.plane_valid_threshold =
            node->declare_parameter<double>("map_builder.plane_valid_threshold", 0.1);

        return std::make_shared<map_builder::IkdTreeMapBuilder>(params);
    }

    throw std::invalid_argument("Unknown map_builder type '" + type + "'.");
}

}  // namespace lio_slam_shaw::factory
