#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"

#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

core::IScanMatcher::SharedPtr ScanMatcherFactory::create(rclcpp::Node* node,
                                                         const Eigen::Isometry3d& T_base_lidar,
                                                         core::IMapBuilder::SharedPtr map_builder) {
    scan_matcher::IkdTreeScanMatcherParams params;
    params.T_base_lidar = T_base_lidar;
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

    return std::make_shared<scan_matcher::IkdTreeScanMatcher>(map_builder, params);
}

}  // namespace lio_slam_shaw::factory
