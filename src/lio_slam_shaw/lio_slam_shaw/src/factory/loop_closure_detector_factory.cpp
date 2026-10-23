#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"

#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/scan_context_loop_closure_detector.hpp"

namespace lio_slam_shaw::factory {

core::ILoopClosureDetector::SharedPtr LoopClosureDetectorFactory::create(rclcpp::Node* node) {
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

    if (type == "scan_context") {
        ScanContextParams sc;
        sc.num_ring =
            node->declare_parameter<int>("backend.loop_closure.scan_context.num_ring", 20);
        sc.num_sector =
            node->declare_parameter<int>("backend.loop_closure.scan_context.num_sector", 60);
        sc.max_radius =
            node->declare_parameter<double>("backend.loop_closure.scan_context.max_radius", 80.0);
        sc.lidar_height =
            node->declare_parameter<double>("backend.loop_closure.scan_context.lidar_height", 2.0);
        sc.num_candidates =
            node->declare_parameter<int>("backend.loop_closure.scan_context.num_candidates", 10);
        sc.sc_dist_threshold = node->declare_parameter<double>(
            "backend.loop_closure.scan_context.sc_dist_threshold", 0.13);
        sc.search_ratio =
            node->declare_parameter<double>("backend.loop_closure.scan_context.search_ratio", 0.1);
        sc.enable_reverse_search = node->declare_parameter<bool>(
            "backend.loop_closure.scan_context.enable_reverse_search", false);

        ScanContextLoopClosureDetectorParams params;
        params.min_time_gap_sec =
            node->declare_parameter<double>("backend.loop_closure.min_time_gap_sec", 30.0);
        params.local_map_keyframe_num =
            node->declare_parameter<int>("backend.loop_closure.local_map_keyframe_num", 15);
        params.icp_downsample_leaf = static_cast<float>(
            node->declare_parameter<double>("backend.loop_closure.icp_downsample_leaf", 0.4));
        params.icp_max_corr_dist =
            node->declare_parameter<double>("backend.loop_closure.icp_max_corr_dist", 1.5);
        params.icp_max_iterations =
            node->declare_parameter<int>("backend.loop_closure.icp_max_iterations", 100);
        params.fitness_score_threshold =
            node->declare_parameter<double>("backend.loop_closure.fitness_score_threshold", 0.3);
        params.noise_sigma_rot =
            node->declare_parameter<double>("backend.loop_closure.noise_sigma_rot", 0.1);
        params.noise_sigma_trans =
            node->declare_parameter<double>("backend.loop_closure.noise_sigma_trans", 0.1);

        return std::make_shared<ScanContextLoopClosureDetector>(sc, params);
    }

    throw std::runtime_error("LoopClosureDetectorFactory: unknown type '" + type + "'");
}

}  // namespace lio_slam_shaw::factory
