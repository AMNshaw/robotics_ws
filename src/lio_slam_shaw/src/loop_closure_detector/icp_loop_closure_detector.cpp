#include "lio_slam_shaw/loop_closure_detector/icp_loop_closure_detector.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <chrono>
#include <limits>

namespace lio_slam_shaw {

IcpLoopClosureDetector::IcpLoopClosureDetector(const IcpLoopClosureDetectorParams& params)
    : params_(params) {}

std::optional<core::LoopConstraint> IcpLoopClosureDetector::detect(
    const core::KeyFrame::SharedPtr& current_keyframe, core::IMapBuilder::SharedPtr map_builder) {
    const auto all_keyframes = map_builder->getAllKeyframes();

    if (all_keyframes.size() < 2) return std::nullopt;

    const auto candidate_opt = findCandidate(current_keyframe, all_keyframes);
    if (!candidate_opt.has_value()) return std::nullopt;
    const auto& candidate = candidate_opt.value();

    const auto local_map = buildLocalMap(current_keyframe, candidate, all_keyframes);
    if (!local_map || local_map->empty()) return std::nullopt;

    pcl::VoxelGrid<core::PointXYZIRT> voxel;
    voxel.setLeafSize(params_.icp_downsample_leaf, params_.icp_downsample_leaf,
                      params_.icp_downsample_leaf);

    auto source_ds = std::make_shared<core::PointCloudIRT>();
    voxel.setInputCloud(current_keyframe->cloud);
    voxel.filter(*source_ds);

    auto target_ds = std::make_shared<core::PointCloudIRT>();
    voxel.setInputCloud(local_map);
    voxel.filter(*target_ds);

    auto source_world = std::make_shared<core::PointCloudIRT>();
    pcl::transformPointCloud(*source_ds, *source_world,
                             current_keyframe->pose.matrix().cast<float>());

    pcl::IterativeClosestPoint<core::PointXYZIRT, core::PointXYZIRT> icp;
    icp.setMaxCorrespondenceDistance(params_.icp_max_corr_dist);
    icp.setMaximumIterations(params_.icp_max_iterations);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setInputSource(source_world);
    icp.setInputTarget(target_ds);

    auto aligned = std::make_shared<core::PointCloudIRT>();
    icp.align(*aligned);

    if (!icp.hasConverged()) return std::nullopt;

    const double score = icp.getFitnessScore();
    if (score > params_.fitness_score_threshold) return std::nullopt;

    const Eigen::Isometry3d T_correction(icp.getFinalTransformation().cast<double>());
    const Eigen::Isometry3d T_w_from = T_correction * current_keyframe->pose;
    const Eigen::Isometry3d T_w_to = candidate->pose;
    const Eigen::Isometry3d relative_pose = T_w_from.inverse() * T_w_to;

    const double scale = 1.0 + score;
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
    cov.block<3, 3>(0, 0) *= std::pow(params_.noise_sigma_rot * scale, 2);
    cov.block<3, 3>(3, 3) *= std::pow(params_.noise_sigma_trans * scale, 2);

    return core::LoopConstraint{current_keyframe->id, candidate->id, relative_pose, cov, score};
}

std::optional<core::KeyFrame::SharedPtr> IcpLoopClosureDetector::findCandidate(
    const core::KeyFrame::SharedPtr& current_keyframe,
    const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const {
    const Eigen::Vector3d curr_pos = current_keyframe->pose.translation();

    core::KeyFrame::SharedPtr best = nullptr;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto& kf : all_keyframes) {
        if (kf->id == current_keyframe->id) continue;

        const double time_diff =
            std::abs(core::getDeltaSec(kf->timestamp, current_keyframe->timestamp));
        if (time_diff < params_.min_time_gap_sec) continue;

        const double dist = (kf->pose.translation() - curr_pos).norm();
        if (dist > params_.search_radius) continue;

        if (dist < best_dist) {
            best_dist = dist;
            best = kf;
        }
    }

    if (!best) return std::nullopt;
    return best;
}

core::PointCloudIRTPtr IcpLoopClosureDetector::buildLocalMap(
    const core::KeyFrame::SharedPtr& current_keyframe, const core::KeyFrame::SharedPtr& candidate,
    const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const {
    const Eigen::Vector3d cand_pos = candidate->pose.translation();

    std::vector<std::pair<double, core::KeyFrame::SharedPtr>> dist_kf;
    dist_kf.reserve(all_keyframes.size());
    for (const auto& kf : all_keyframes) {
        const double time_diff =
            std::abs(core::getDeltaSec(kf->timestamp, current_keyframe->timestamp));
        if (time_diff < params_.min_time_gap_sec) {
            continue;
        }
        const double d = (kf->pose.translation() - cand_pos).norm();
        dist_kf.emplace_back(d, kf);
    }
    std::sort(dist_kf.begin(), dist_kf.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    const int n = std::min(static_cast<int>(dist_kf.size()), params_.local_map_keyframe_num);

    auto local_map = std::make_shared<core::PointCloudIRT>();
    for (int i = 0; i < n; ++i) {
        const auto& kf = dist_kf[i].second;
        if (!kf->cloud || kf->cloud->empty()) continue;

        core::PointCloudIRT cloud_world;
        pcl::transformPointCloud(*kf->cloud, cloud_world, kf->pose.matrix().cast<float>());
        *local_map += cloud_world;
    }

    return local_map;
}

}  // namespace lio_slam_shaw
