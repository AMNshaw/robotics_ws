#include "lio_slam_shaw/core/backend.hpp"

namespace lio_slam_shaw::core {

BackEnd::BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
                 ILoopClosureDetector::SharedPtr loop_closure_detector)
    : map_builder_(std::move(map_builder)),
      map_optimizer_(std::move(map_optimizer)),
      loop_closure_detector_(std::move(loop_closure_detector)) {}

void BackEnd::processFrame(const LidarFrame::SharedPtr& frame) {
    auto keyframe_opt = map_builder_->addFrame(frame);
    if (!keyframe_opt.has_value()) return;
    const auto& keyframe = keyframe_opt.value();

    map_optimizer_->addKeyframe(keyframe->id, frame->matched_result);

    auto loop_opt = loop_closure_detector_->detect(keyframe, map_builder_);
    if (!loop_opt.has_value()) return;

    map_optimizer_->addLoopConstraint(loop_opt.value());
    auto corrected_poses = map_optimizer_->optimize();

    pending_corrected_poses_ = corrected_poses;
    for (const auto& [id, pose] : corrected_poses) {
        if (id == keyframe->id) {
            pending_correction_ = pose * frame->matched_result.pose.inverse();
            break;
        }
    }
}

std::optional<Eigen::Isometry3d> BackEnd::updateGlobalCorrection() {
    auto result = pending_correction_;
    pending_correction_ = std::nullopt;
    return result;
}

void BackEnd::updateMap() {
    if (pending_corrected_poses_.empty()) return;
    map_builder_->updateKeyframePoses(pending_corrected_poses_);
    pending_corrected_poses_.clear();
}

}  // namespace lio_slam_shaw::core
