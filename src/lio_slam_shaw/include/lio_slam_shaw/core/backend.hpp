#ifndef LIO_SLAM_SHAW__CORE__BACKEND_HPP_
#define LIO_SLAM_SHAW__CORE__BACKEND_HPP_

#include <optional>
#include <utility>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_map_optimizer.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"

namespace lio_slam_shaw::core {

class BackEnd {
public:
    using SharedPtr = std::shared_ptr<BackEnd>;
    using ConstSharedPtr = std::shared_ptr<const BackEnd>;

    BackEnd(IMapBuilder::SharedPtr map_builder, IMapOptimizer::SharedPtr map_optimizer,
            ILoopClosureDetector::SharedPtr loop_closure_detector);
    ~BackEnd() = default;

    void processKeyframe(const KeyFrame::SharedPtr& frame);

    std::optional<Eigen::Isometry3d> updateGlobalCorrection();

    void updateMap();

private:
    IMapBuilder::SharedPtr map_builder_;
    IMapOptimizer::SharedPtr map_optimizer_;
    ILoopClosureDetector::SharedPtr loop_closure_detector_;

    std::optional<Eigen::Isometry3d> pending_correction_;
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> pending_corrected_poses_;
};

}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__BACKEND_HPP_