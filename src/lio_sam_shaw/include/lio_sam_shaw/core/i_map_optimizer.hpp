#ifndef LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_
#define LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_

#include <memory>
#include <vector>

#include "lio_sam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class IMapOptimizer {
public:
    using SharedPtr = std::shared_ptr<IMapOptimizer>;
    using ConstSharedPtr = std::shared_ptr<const IMapOptimizer>;

    virtual ~IMapOptimizer() = default;

    // 加入新的 keyframe 連結（odometry edge），回傳優化後的當前幀 pose
    // matched_result.pose 是全局座標系下的絕對位姿，內部計算相對 pose 後建構 BetweenFactor
    virtual NavState addKeyframe(uint64_t keyframe_id, const ScanMatchResult& matched_result) = 0;

    // 加入 loop closure 約束，回傳所有 keyframe 的修正後 pose（用於更新 MapBuilder）
    virtual std::vector<std::pair<uint64_t, Eigen::Isometry3d>> addLoopConstraint(
        const LoopConstraint& constraint) = 0;

    // 查詢指定 keyframe 優化後的 pose
    virtual NavState getKeyframePose(uint64_t keyframe_id) const = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_