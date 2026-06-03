#include "lio_slam_shaw/core/backend.hpp"

namespace lio_slam_shaw::core {

BackEnd::BackEnd(IGlobalMapBuilder::SharedPtr global_map, IMapOptimizer::SharedPtr map_optimizer,
                 ILoopClosureDetector::SharedPtr loop_closure_detector)
    : global_map_(std::move(global_map)),
      map_optimizer_(std::move(map_optimizer)),
      loop_closure_detector_(std::move(loop_closure_detector)) {}

std::optional<Keyframe::SharedPtr> BackEnd::tryAddKeyframe(const LidarFrame::SharedPtr& frame) {
    return global_map_->addKeyFrame(frame);
}

void BackEnd::processKeyframe(const Keyframe::SharedPtr& keyframe) {
    map_optimizer_->addKeyframe(keyframe->id, keyframe->matched_result);

    // Update keyframe pose to optimizer's map-frame estimate immediately
    keyframe->pose = map_optimizer_->getKeyframePose(keyframe->id).pose;

    auto loop_opt = loop_closure_detector_->detect(keyframe, global_map_);
    if (!loop_opt.has_value()) return;

    const auto& lc = loop_opt.value();
    map_optimizer_->addLoopConstraint(lc);

    // Store loop edge for visualization
    {
        auto from_kf = global_map_->getKeyframe(lc.from_id);
        auto to_kf = global_map_->getKeyframe(lc.to_id);
        if (from_kf && to_kf) {
            std::lock_guard<std::mutex> lock(loop_edges_mutex_);
            loop_edges_.push_back(LoopEdge{lc.from_id, lc.to_id,
                                           from_kf.value()->pose.translation(),
                                           to_kf.value()->pose.translation(), lc.fitness_score});
        }
    }

    auto corrected_poses = map_optimizer_->optimize();

    pending_corrected_poses_ = corrected_poses;
    const auto& [last_id, corrected_pose] = corrected_poses.back();
    pending_correction_ = corrected_pose * keyframe->matched_result.pose.inverse();
}

bool BackEnd::updateGlobalCorrection() {
    if (!pending_correction_.has_value() || pending_corrected_poses_.empty()) {
        pending_correction_ = std::nullopt;
        pending_corrected_poses_.clear();
        return false;
    }

    T_map_odom_ = pending_correction_.value();  // full T_map_odom, not delta
    pending_correction_ = std::nullopt;

    global_map_->updateKeyframePoses(pending_corrected_poses_);
    pending_corrected_poses_.clear();
    return true;
}

Eigen::Isometry3d BackEnd::getGlobalCorrection() const { return T_map_odom_; }

std::vector<Keyframe::SharedPtr> BackEnd::getAllKeyframes() const {
    return global_map_->getAllKeyframes();
}

std::vector<std::pair<uint64_t, Eigen::Isometry3d>> BackEnd::getAllKeyframePoses() const {
    auto keyframes = global_map_->getAllKeyframes();
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> poses;
    poses.reserve(keyframes.size());
    for (const auto& kf : keyframes) {
        poses.emplace_back(kf->id, kf->pose);
    }
    return poses;
}

std::vector<LoopEdge> BackEnd::getLoopEdges() const {
    std::lock_guard<std::mutex> lock(loop_edges_mutex_);
    return loop_edges_;
}

PointCloudIRTPtr BackEnd::getGlobalMap() const { return global_map_->getGlobalMap(); }

}  // namespace lio_slam_shaw::core
