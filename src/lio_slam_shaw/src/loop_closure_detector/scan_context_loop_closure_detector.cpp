#include "lio_slam_shaw/loop_closure_detector/scan_context_loop_closure_detector.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/impl/correspondence_estimation.hpp>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/search.hpp>

namespace lio_slam_shaw {

ScanContextLoopClosureDetector::ScanContextLoopClosureDetector(
    const ScanContextParams& sc_params, const ScanContextLoopClosureDetectorParams& params)
    : sc_manager_(std::make_unique<ScanContextManager>(sc_params)), params_(params) {}

// ---------------------------------------------------------------------------

std::optional<core::LoopConstraint> ScanContextLoopClosureDetector::detect(
    const core::Keyframe::SharedPtr& current_keyframe,
    core::IGlobalMapBuilder::SharedPtr global_map) {
    // 0. Build history from descriptors_ (source of truth) + time filter
    std::vector<core::Keyframe::SharedPtr> history_keyframes;
    history_keyframes.reserve(descriptors_.size());
    for (const auto& [id, _] : descriptors_) {
        auto kf_opt = global_map->getKeyframe(id);
        if (!kf_opt.has_value()) continue;
        const double dt =
            std::abs(core::getDeltaSec(kf_opt.value()->timestamp, current_keyframe->timestamp));
        if (dt >= params_.min_time_gap_sec) history_keyframes.push_back(kf_opt.value());
    }

    // 1. SC place recognition
    const auto match = findLoopCandidate(current_keyframe, history_keyframes);
    if (!match.has_value()) return std::nullopt;
    const auto& [candidate, yaw_diff_rad] = match.value();

    // 2. Build local map around candidate (from same history set)
    const auto local_map = buildLocalMap(candidate, history_keyframes);
    if (!local_map || local_map->empty()) return std::nullopt;

    // 3. GICP verification
    return refineWithICP(current_keyframe, candidate, yaw_diff_rad, local_map);
}

// ---------------------------------------------------------------------------

std::optional<std::pair<core::Keyframe::SharedPtr, float>>
ScanContextLoopClosureDetector::findLoopCandidate(
    const core::Keyframe::SharedPtr& current_keyframe,
    const std::vector<core::Keyframe::SharedPtr>& history_keyframes) {
    const uint64_t kf_id = current_keyframe->id;

    // Compute and store descriptor
    ScDescriptor desc = sc_manager_->getDescriptor(current_keyframe->cloud_body);
    descriptors_.emplace(kf_id, desc);

    // Build candidate set from time-filtered history
    std::unordered_map<uint64_t, ScDescriptor> candidates;
    for (const auto& kf : history_keyframes) {
        auto it = descriptors_.find(kf->id);
        if (it != descriptors_.end()) candidates.emplace(kf->id, it->second);
    }

    // SC matching
    const auto result = sc_manager_->findClosest(desc, candidates);
    if (!result.has_value()) return std::nullopt;
    const auto [candidate_id, yaw_diff_rad] = result.value();

    // Find the keyframe pointer
    for (const auto& kf : history_keyframes) {
        if (kf->id == candidate_id) return std::make_pair(kf, yaw_diff_rad);
    }
    return std::nullopt;
}

// ---------------------------------------------------------------------------

core::PointCloudIRTPtr ScanContextLoopClosureDetector::buildLocalMap(
    const core::Keyframe::SharedPtr& candidate,
    const std::vector<core::Keyframe::SharedPtr>& history_keyframes) const {
    std::vector<std::pair<double, core::Keyframe::SharedPtr>> by_dist;
    by_dist.reserve(history_keyframes.size());
    for (const auto& kf : history_keyframes) {
        double d = (kf->pose.translation() - candidate->pose.translation()).norm();
        by_dist.emplace_back(d, kf);
    }
    std::sort(by_dist.begin(), by_dist.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    auto cloud = std::make_shared<core::PointCloudIRT>();
    int count = 0;
    for (const auto& entry : by_dist) {
        if (count >= params_.local_map_keyframe_num) break;
        const auto& kf = entry.second;
        if (!kf->cloud_body || kf->cloud_body->empty()) continue;
        core::PointCloudIRT transformed;
        pcl::transformPointCloud(*kf->cloud_body, transformed, kf->pose.matrix().cast<float>());
        *cloud += transformed;
        ++count;
    }
    return cloud;
}

// ---------------------------------------------------------------------------

std::optional<core::LoopConstraint> ScanContextLoopClosureDetector::refineWithICP(
    const core::Keyframe::SharedPtr& current_keyframe, const core::Keyframe::SharedPtr& candidate,
    float yaw_diff_rad, const core::PointCloudIRTPtr& local_map) const {
    // Downsample
    pcl::VoxelGrid<core::PointXYZIRT> voxel;
    voxel.setLeafSize(params_.icp_downsample_leaf, params_.icp_downsample_leaf,
                      params_.icp_downsample_leaf);

    auto source_ds = std::make_shared<core::PointCloudIRT>();
    voxel.setInputCloud(current_keyframe->cloud_body);
    voxel.filter(*source_ds);

    auto target_ds = std::make_shared<core::PointCloudIRT>();
    voxel.setInputCloud(local_map);
    voxel.filter(*target_ds);

    // Initial guess: place current at candidate's position, with yaw adjusted by SC difference
    Eigen::Isometry3d T_init = candidate->pose;
    T_init.linear() = Eigen::AngleAxisd(static_cast<double>(yaw_diff_rad), Eigen::Vector3d::UnitZ())
                          .toRotationMatrix() *
                      candidate->pose.linear();

    auto source_world = std::make_shared<core::PointCloudIRT>();
    pcl::transformPointCloud(*source_ds, *source_world, T_init.matrix().cast<float>());

    // GICP
    pcl::GeneralizedIterativeClosestPoint<core::PointXYZIRT, core::PointXYZIRT> gicp;
    gicp.setMaxCorrespondenceDistance(params_.icp_max_corr_dist);
    gicp.setMaximumIterations(params_.icp_max_iterations);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    gicp.setInputSource(source_world);
    gicp.setInputTarget(target_ds);

    auto aligned = std::make_shared<core::PointCloudIRT>();
    gicp.align(*aligned);

    if (!gicp.hasConverged()) return std::nullopt;
    const double score = gicp.getFitnessScore();
    if (score > params_.fitness_score_threshold) return std::nullopt;

    // Relative pose
    const Eigen::Isometry3d T_gicp(gicp.getFinalTransformation().cast<double>());
    const Eigen::Isometry3d T_w_from = T_gicp * T_init;

    const Eigen::Isometry3d relative_pose = T_w_from.inverse() * candidate->pose;

    const double scale = 1.0 + score;
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
    cov.block<3, 3>(0, 0) *= std::pow(params_.noise_sigma_rot * scale, 2.0);
    cov.block<3, 3>(3, 3) *= std::pow(params_.noise_sigma_trans * scale, 2.0);

    std::clog << "[SCLoop] kf=" << current_keyframe->id << " -> " << candidate->id
              << " score=" << score << " yaw=" << yaw_diff_rad << "rad\n";

    return core::LoopConstraint{current_keyframe->id, candidate->id, relative_pose, cov, score};
}

}  // namespace lio_slam_shaw
