#include "lio_slam_shaw/core/backend.hpp"

namespace lio_slam_shaw::core {

BackEnd::BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
                 ILoopClosureDetector::SharedPtr loop_closure_detector)
    : map_builder_(std::move(map_builder)),
      map_optimizer_(std::move(map_optimizer)),
      loop_closure_detector_(std::move(loop_closure_detector)) {}

void BackEnd::processKeyframe(const KeyFrame::SharedPtr& keyframe) {
    map_optimizer_->addKeyframe(keyframe->id, keyframe->matched_result);

    auto loop_opt = loop_closure_detector_->detect(keyframe, map_builder_);
    if (!loop_opt.has_value()) return;

    map_optimizer_->addLoopConstraint(loop_opt.value());
    auto corrected_poses = map_optimizer_->optimize();

    pending_corrected_poses_ = corrected_poses;
    const auto& [last_id, corrected_pose] = corrected_poses.back();
    pending_correction_ = corrected_pose * keyframe->matched_result.pose.inverse();
}

std::optional<Eigen::Isometry3d> BackEnd::updateGlobalCorrection() {
    auto result = pending_correction_;
    pending_correction_ = std::nullopt;

    if (pending_corrected_poses_.empty()) return std::nullopt;
    map_builder_->updateKeyframePoses(pending_corrected_poses_);
    pending_corrected_poses_.clear();

    return result;
}

void BackEnd::updateMap() { map_builder_->updateMap(); }

}  // namespace lio_slam_shaw::core
