#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"

#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

core::IScanMatcher::SharedPtr ScanMatcherFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar,
    std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map) {
    const std::string type = node->declare_parameter<std::string>("scan_matcher.type", "ikd_tree");

    if (type == "ikd_tree") {
        scan_matcher::IkdTreeScanMatcherParams params;
        params.T_base_lidar = T_base_lidar;
        params.k_neighbors = node->declare_parameter<int>("scan_matcher.k_neighbors", 5);
        params.max_iterations = node->declare_parameter<int>("scan_matcher.max_iterations", 30);
        params.convergence_threshold =
            node->declare_parameter<double>("scan_matcher.convergence_threshold", 1e-5);
        params.min_valid_points = node->declare_parameter<int>("scan_matcher.min_valid_points", 50);
        params.degenerate_threshold =
            node->declare_parameter<double>("scan_matcher.degenerate_threshold", 100.0);
        params.max_search_dist =
            node->declare_parameter<double>("scan_matcher.max_search_dist", 2.0);
        params.min_plane_points = node->declare_parameter<int>("scan_matcher.min_plane_points", 5);
        params.plane_valid_threshold =
            node->declare_parameter<double>("scan_matcher.plane_valid_threshold", 0.1);
        params.rot_regularization_sigma =
            node->declare_parameter<double>("scan_matcher.rot_regularization_sigma", 0.0);

        return std::make_shared<scan_matcher::IkdTreeScanMatcher>(local_map, params);
    }

    throw std::invalid_argument("Unknown scan_matcher type '" + type + "'.");
}

}  // namespace lio_slam_shaw::factory
