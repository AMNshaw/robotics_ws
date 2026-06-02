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

    auto loop_opt = loop_closure_detector_->detect(keyframe, global_map_);
    if (!loop_opt.has_value()) return;

    map_optimizer_->addLoopConstraint(loop_opt.value());
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

    T_map_odom_ = pending_correction_.value() * T_map_odom_;
    pending_correction_ = std::nullopt;

    global_map_->updateKeyframePoses(pending_corrected_poses_);
    pending_corrected_poses_.clear();
    return true;
}

Eigen::Isometry3d BackEnd::getGlobalCorrection() const { return T_map_odom_; }

}  // namespace lio_slam_shaw::core
