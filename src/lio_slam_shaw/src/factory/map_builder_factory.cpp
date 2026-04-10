#include "lio_slam_shaw/factory/map_builder_factory.hpp"

#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

namespace lio_slam_shaw::factory {

core::IMapBuilder::SharedPtr MapBuilderFactory::create(rclcpp::Node::SharedPtr node) {
    std::string type = node->declare_parameter<std::string>("map_builder_type", "ikd_tree");

    map_builder::IkdTreeMapBuilderParams params;
    params.keyframe_distance_threshold = node->declare_parameter<double>(
        "map_builder.keyframe_distance_threshold", params.keyframe_distance_threshold);
    params.keyframe_angle_threshold = node->declare_parameter<double>(
        "map_builder.keyframe_angle_threshold", params.keyframe_angle_threshold);
    params.ikd_delete_param = static_cast<float>(
        node->declare_parameter<double>("map_builder.ikd_delete_param", params.ikd_delete_param));
    params.ikd_balance_param = static_cast<float>(
        node->declare_parameter<double>("map_builder.ikd_balance_param", params.ikd_balance_param));
    params.ikd_downsample_size = static_cast<float>(node->declare_parameter<double>(
        "map_builder.ikd_downsample_size", params.ikd_downsample_size));
    params.max_search_dist =
        node->declare_parameter<double>("map_builder.max_search_dist", params.max_search_dist);
    params.min_plane_points =
        node->declare_parameter<int>("map_builder.min_plane_points", params.min_plane_points);
    params.plane_valid_threshold = node->declare_parameter<double>(
        "map_builder.plane_valid_threshold", params.plane_valid_threshold);

    if (type != "ikd_tree") {
        RCLCPP_WARN(node->get_logger(), "Unknown map_builder_type '%s', falling back to ikd_tree.",
                    type.c_str());
    }
    return std::make_shared<map_builder::IkdTreeMapBuilder>(params);
}

}  // namespace lio_slam_shaw::factory
