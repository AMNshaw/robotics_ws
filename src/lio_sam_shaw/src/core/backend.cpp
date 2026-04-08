#include "lio_sam_shaw/core/backend.hpp"

namespace lio_sam_shaw::core {

BackEnd::BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
                 ILoopClosureDetector::SharedPtr loop_closure_detector)
    : map_builder_(std::move(map_builder)),
      map_optimizer_(std::move(map_optimizer)),
      loop_closure_detector_(std::move(loop_closure_detector)) {}

std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> BackEnd::processFrame(
    const LidarFrame::SharedPtr& frame) {
    // 1. 由 MapBuilder 判斷是否為 keyframe，不是則直接跳過
    auto keyframe_opt = map_builder_->addKeyframe(frame);
    if (!keyframe_opt.has_value()) return std::nullopt;
    const auto& keyframe = keyframe_opt.value();

    // 2. 加入 odometry edge，優化當前 keyframe pose
    map_optimizer_->addKeyframe(keyframe->id, frame->matched_result);

    // 3. 偵測 loop closure，若有則加入約束並全局優化
    auto loop_opt = loop_closure_detector_->detect(keyframe, *map_builder_);
    if (loop_opt.has_value()) {
        auto corrected_poses = map_optimizer_->addLoopConstraint(loop_opt.value());
        map_builder_->updateKeyframePoses(corrected_poses);

        // 找出當前 keyframe 修正後的 pose，回傳給 SlamProcessor 通知前端
        for (const auto& [id, pose] : corrected_poses) {
            if (id == keyframe->id) {
                return std::make_pair(frame->matched_result.pose, pose);
            }
        }
    }
    return std::nullopt;
}

}  // namespace lio_sam_shaw::core
