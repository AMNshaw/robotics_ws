#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"

#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/ikd_tree_loop_closure_detector.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

core::ILoopClosureDetector::SharedPtr LoopClosureDetectorFactory::create(
    rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar) {
    const std::string type =
        node->declare_parameter<std::string>("backend.loop_closure.type", "icp");

    if (type == "icp") {
        IcpLoopClosureDetectorParams params;

        params.search_radius =
            node->declare_parameter<double>("backend.loop_closure.search_radius", 15.0);

        params.min_time_gap_sec =
            node->declare_parameter<double>("backend.loop_closure.min_time_gap_sec", 30.0);

        params.local_map_keyframe_num =
            node->declare_parameter<int>("backend.loop_closure.local_map_keyframe_num", 25);

        params.icp_downsample_leaf = static_cast<float>(
            node->declare_parameter<double>("backend.loop_closure.icp_downsample_leaf", 0.4));

        params.icp_max_corr_dist =
            node->declare_parameter<double>("backend.loop_closure.icp_max_corr_dist", 0.3);

        params.icp_max_iterations =
            node->declare_parameter<int>("backend.loop_closure.icp_max_iterations", 100);

        params.fitness_score_threshold =
            node->declare_parameter<double>("backend.loop_closure.fitness_score_threshold", 0.3);

        params.noise_sigma_rot =
            node->declare_parameter<double>("backend.loop_closure.noise_sigma_rot", 0.1);

        params.noise_sigma_trans =
            node->declare_parameter<double>("backend.loop_closure.noise_sigma_trans", 0.1);

        return std::make_shared<IcpLoopClosureDetector>(params);
    }

    if (type == "ikd_tree") {
        scan_matcher::IkdTreeScanMatcherParams scan_matcher_params;
        scan_matcher_params.k_neighbors =
            node->declare_parameter<int>("backend.loop_closure.scan_matcher.k_neighbors", 5);
        scan_matcher_params.max_iterations =
            node->declare_parameter<int>("backend.loop_closure.scan_matcher.max_iterations", 100);
        scan_matcher_params.convergence_threshold = node->declare_parameter<double>(
            "backend.loop_closure.scan_matcher.convergence_threshold", 1e-6);
        scan_matcher_params.min_valid_points =
            node->declare_parameter<int>("backend.loop_closure.scan_matcher.min_valid_points", 50);
        scan_matcher_params.degenerate_threshold = node->declare_parameter<double>(
            "backend.loop_closure.scan_matcher.degenerate_threshold", 100.0);
        scan_matcher_params.max_search_dist = node->declare_parameter<double>(
            "backend.loop_closure.scan_matcher.max_search_dist", 2.0);
        scan_matcher_params.min_plane_points =
            node->declare_parameter<int>("backend.loop_closure.scan_matcher.min_plane_points", 5);
        scan_matcher_params.plane_valid_threshold = node->declare_parameter<double>(
            "backend.loop_closure.scan_matcher.plane_valid_threshold", 0.1);

        scan_matcher_params.T_base_lidar = T_base_lidar;

        IkdTreeLoopClosureDetectorParams loop_closure_params;
        loop_closure_params.search_radius =
            node->declare_parameter<double>("backend.loop_closure.search_radius", 15.0);
        loop_closure_params.min_time_gap_sec =
            node->declare_parameter<double>("backend.loop_closure.min_time_gap_sec", 30.0);

        return std::make_shared<IkdTreeLoopClosureDetector>(scan_matcher_params,
                                                            loop_closure_params);
    }

    throw std::runtime_error("LoopClosureDetectorFactory: unknown type '" + type + "'");
}

}  // namespace lio_slam_shaw::factory
