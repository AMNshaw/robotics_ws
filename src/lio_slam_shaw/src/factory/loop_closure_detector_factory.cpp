#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"

#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/ikd_tree_loop_closure_detector.hpp"

namespace lio_slam_shaw::factory {

core::ILoopClosureDetector::SharedPtr LoopClosureDetectorFactory::create(
    rclcpp::Node::SharedPtr node, core::IScanMatcher::SharedPtr scan_matcher) {
    const std::string type =
        node->declare_parameter<std::string>("loop_closure_detector_type", "icp");

    if (type == "icp") {
        IcpLoopClosureDetectorParams params;

        params.search_radius = node->declare_parameter<double>("loop_closure.search_radius", 15.0);

        params.min_time_gap_sec =
            node->declare_parameter<double>("loop_closure.min_time_gap_sec", 30.0);

        params.local_map_keyframe_num =
            node->declare_parameter<int>("loop_closure.local_map_keyframe_num", 25);

        params.icp_downsample_leaf = static_cast<float>(
            node->declare_parameter<double>("loop_closure.icp_downsample_leaf", 0.4));

        params.icp_max_corr_dist =
            node->declare_parameter<double>("loop_closure.icp_max_corr_dist", 0.3);

        params.icp_max_iterations =
            node->declare_parameter<int>("loop_closure.icp_max_iterations", 100);

        params.fitness_score_threshold =
            node->declare_parameter<double>("loop_closure.fitness_score_threshold", 0.3);

        params.noise_sigma_rot =
            node->declare_parameter<double>("loop_closure.noise_sigma_rot", 0.1);

        params.noise_sigma_trans =
            node->declare_parameter<double>("loop_closure.noise_sigma_trans", 0.1);

        return std::make_shared<IcpLoopClosureDetector>(params);
    }

    if (type == "ikd_tree") {
        if (!scan_matcher) {
            throw std::runtime_error(
                "LoopClosureDetectorFactory: ikd_tree type requires a scan_matcher");
        }
        IkdTreeLoopClosureDetectorParams params;
        params.search_radius = node->declare_parameter<double>("loop_closure.search_radius", 15.0);
        params.min_time_gap_sec =
            node->declare_parameter<double>("loop_closure.min_time_gap_sec", 30.0);
        return std::make_shared<IkdTreeLoopClosureDetector>(scan_matcher, params);
    }

    throw std::runtime_error("LoopClosureDetectorFactory: unknown type '" + type + "'");
}

}  // namespace lio_slam_shaw::factory
