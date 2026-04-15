#include "lio_slam_shaw/core/backend.hpp"

namespace lio_slam_shaw::core {

BackEnd::BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
                 ILoopClosureDetector::SharedPtr loop_closure_detector)
    : map_builder_(std::move(map_builder)),
      map_optimizer_(std::move(map_optimizer)),
      loop_closure_detector_(std::move(loop_closure_detector)) {}

void BackEnd::processKeyframe(const Keyframe::SharedPtr& keyframe) {
    map_optimizer_->addKeyframe(keyframe->id, keyframe->matched_result);

    auto loop_opt = loop_closure_detector_->detect(keyframe, map_builder_);
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

    map_builder_->updateKeyframePoses(pending_corrected_poses_);
    pending_corrected_poses_.clear();
    return true;
}

Eigen::Isometry3d BackEnd::getGlobalCorrection() const { return T_map_odom_; }

void BackEnd::updateMap() { map_builder_->updateMap(); }

}  // namespace lio_slam_shaw::core
