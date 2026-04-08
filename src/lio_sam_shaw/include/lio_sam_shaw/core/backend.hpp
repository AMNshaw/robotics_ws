#ifndef LIO_SAM_SHAW__CORE__BACKEND_HPP_
#define LIO_SAM_SHAW__CORE__BACKEND_HPP_

#include <optional>
#include <utility>

#include "lio_sam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_sam_shaw/core/i_map_builder.hpp"
#include "lio_sam_shaw/core/i_map_optimizer.hpp"
#include "lio_sam_shaw/core/lidar_frame.hpp"

namespace lio_sam_shaw::core {

class BackEnd {
public:
    using SharedPtr = std::shared_ptr<BackEnd>;
    using ConstSharedPtr = std::shared_ptr<const BackEnd>;

    BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
            ILoopClosureDetector::SharedPtr loop_closure_detector);
    ~BackEnd() = default;

    // 回傳 loop closure 的修正對 {original_pose, corrected_pose}
    // 沒有 loop closure 則回傳 nullopt
    std::optional<std::pair<Eigen::Isometry3d, Eigen::Isometry3d>> processFrame(
        const LidarFrame::SharedPtr& frame);

private:
    IMapBuilder::SharedPtr map_builder_;
    IMapOptimizer::SharedPtr map_optimizer_;
    ILoopClosureDetector::SharedPtr loop_closure_detector_;
};

}  // namespace lio_sam_shaw::core
#endif  // LIO_SAM_SHAW__CORE__BACKEND_HPP_