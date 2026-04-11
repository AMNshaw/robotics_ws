#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"

#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

core::IScanMatcher::SharedPtr ScanMatcherFactory::create(rclcpp::Node::SharedPtr node,
                                                         core::IMapBuilder::SharedPtr map_builder) {
    scan_matcher::IkdTreeScanMatcherParams params;
    params.k_neighbors =
        node->declare_parameter<int>("scan_matcher.k_neighbors", params.k_neighbors);
    params.max_iterations =
        node->declare_parameter<int>("scan_matcher.max_iterations", params.max_iterations);
    params.convergence_threshold = node->declare_parameter<double>(
        "scan_matcher.convergence_threshold", params.convergence_threshold);
    params.min_valid_points =
        node->declare_parameter<int>("scan_matcher.min_valid_points", params.min_valid_points);
    params.degenerate_threshold = node->declare_parameter<double>(
        "scan_matcher.degenerate_threshold", params.degenerate_threshold);
    params.T_body_lidar_trans = node->declare_parameter<std::vector<double>>(
        "scan_matcher.T_body_lidar_trans", params.T_body_lidar_trans);
    params.T_body_lidar_rot = node->declare_parameter<std::vector<double>>(
        "scan_matcher.T_body_lidar_rot", params.T_body_lidar_rot);
    return std::make_shared<scan_matcher::IkdTreeScanMatcher>(map_builder, params);
}

}  // namespace lio_slam_shaw::factory
