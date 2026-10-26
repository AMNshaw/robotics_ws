#include "lio_slam_shaw_ros2/factory/loop_closure_detector_factory.hpp"

#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/scan_context_loop_closure_detector.hpp"

namespace lio_slam_shaw_ros2::factory {

core::ILoopClosureDetector::SharedPtr RosLoopClosureDetectorFactory::create(
    loop_closure_detector::LoopClosureDetectorType type) const {
    switch (type) {
        case loop_closure_detector::LoopClosureDetectorType::ICP:
            return createICP();
        case loop_closure_detector::LoopClosureDetectorType::SCAN_CONTEXT:
            return createScanContext();
    }
    throw std::runtime_error("LoopClosureDetectorFactory: unknown type");
}

core::ILoopClosureDetector::SharedPtr RosLoopClosureDetectorFactory::createScanContext() const {
    lio_slam_shaw::Config sc;
    sc.num_ring = node_->declare_parameter<int>("backend.loop_closure.scan_context.num_ring", 20);
    sc.num_sector =
        node_->declare_parameter<int>("backend.loop_closure.scan_context.num_sector", 60);
    sc.max_radius =
        node_->declare_parameter<double>("backend.loop_closure.scan_context.max_radius", 80.0);
    sc.lidar_height =
        node_->declare_parameter<double>("backend.loop_closure.scan_context.lidar_height", 2.0);
    sc.num_candidates =
        node_->declare_parameter<int>("backend.loop_closure.scan_context.num_candidates", 10);
    sc.sc_dist_threshold = node_->declare_parameter<double>(
        "backend.loop_closure.scan_context.sc_dist_threshold", 0.13);
    sc.search_ratio =
        node_->declare_parameter<double>("backend.loop_closure.scan_context.search_ratio", 0.1);
    sc.enable_reverse_search = node_->declare_parameter<bool>(
        "backend.loop_closure.scan_context.enable_reverse_search", false);

    lio_slam_shaw::ScanContextLoopClosureDetectorParams params;
    params.min_time_gap_sec =
        node_->declare_parameter<double>("backend.loop_closure.min_time_gap_sec", 30.0);
    params.local_map_keyframe_num =
        node_->declare_parameter<int>("backend.loop_closure.local_map_keyframe_num", 15);
    params.icp_downsample_leaf = static_cast<float>(
        node_->declare_parameter<double>("backend.loop_closure.icp_downsample_leaf", 0.4));
    params.icp_max_corr_dist =
        node_->declare_parameter<double>("backend.loop_closure.icp_max_corr_dist", 1.5);
    params.icp_max_iterations =
        node_->declare_parameter<int>("backend.loop_closure.icp_max_iterations", 100);
    params.fitness_score_threshold =
        node_->declare_parameter<double>("backend.loop_closure.fitness_score_threshold", 0.3);
    params.noise_sigma_rot =
        node_->declare_parameter<double>("backend.loop_closure.noise_sigma_rot", 0.1);
    params.noise_sigma_trans =
        node_->declare_parameter<double>("backend.loop_closure.noise_sigma_trans", 0.1);

    return std::make_shared<lio_slam_shaw::ScanContextLoopClosureDetector>(sc, params);
}

core::ILoopClosureDetector::SharedPtr RosLoopClosureDetectorFactory::createICP() const {
    lio_slam_shaw::IcpLoopClosureDetectorParams params;

    params.search_radius =
        node_->declare_parameter<double>("backend.loop_closure.search_radius", 15.0);

    params.min_time_gap_sec =
        node_->declare_parameter<double>("backend.loop_closure.min_time_gap_sec", 30.0);

    params.local_map_keyframe_num =
        node_->declare_parameter<int>("backend.loop_closure.local_map_keyframe_num", 25);

    params.icp_downsample_leaf = static_cast<float>(
        node_->declare_parameter<double>("backend.loop_closure.icp_downsample_leaf", 0.4));

    params.icp_max_corr_dist =
        node_->declare_parameter<double>("backend.loop_closure.icp_max_corr_dist", 0.3);

    params.icp_max_iterations =
        node_->declare_parameter<int>("backend.loop_closure.icp_max_iterations", 100);

    params.fitness_score_threshold =
        node_->declare_parameter<double>("backend.loop_closure.fitness_score_threshold", 0.3);

    params.noise_sigma_rot =
        node_->declare_parameter<double>("backend.loop_closure.noise_sigma_rot", 0.1);

    params.noise_sigma_trans =
        node_->declare_parameter<double>("backend.loop_closure.noise_sigma_trans", 0.1);

    return std::make_shared<lio_slam_shaw::IcpLoopClosureDetector>(params);
}

}  // namespace lio_slam_shaw_ros2::factory
