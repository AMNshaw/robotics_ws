#ifndef LIO_SLAM_SHAW__CORE__BACKEND_HPP_
#define LIO_SLAM_SHAW__CORE__BACKEND_HPP_

#include <mutex>
#include <optional>
#include <utility>

#include "lio_slam_shaw/core/i_global_map_builder.hpp"
#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_map_optimizer.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/visualization_types.hpp"

namespace lio_slam_shaw::core {

class BackEnd {
public:
    using SharedPtr = std::shared_ptr<BackEnd>;
    using ConstSharedPtr = std::shared_ptr<const BackEnd>;

    BackEnd(IGlobalMapBuilder::SharedPtr global_map, IMapOptimizer::SharedPtr map_optimizer,
            ILoopClosureDetector::SharedPtr loop_closure_detector);
    ~BackEnd() = default;

    /// Check if a frame qualifies as a keyframe. Returns the keyframe if yes.
    std::optional<Keyframe::SharedPtr> tryAddKeyframe(const LidarFrame::SharedPtr& frame);

    void processKeyframe(const Keyframe::SharedPtr& frame);

    bool updateGlobalCorrection();
    Eigen::Isometry3d getGlobalCorrection() const;

    /// Access keyframes from the global map (thread-safe read via caller's lock).
    std::vector<Keyframe::SharedPtr> getAllKeyframes() const;

    /// Get lightweight keyframe poses (id, pose) for visualization.
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> getAllKeyframePoses() const;

    /// Get all detected loop closure edges (for visualization).
    std::vector<LoopEdge> getLoopEdges() const;

    /// Assemble the full global map point cloud.
    PointCloudIRTPtr getGlobalMap() const;

private:
    IGlobalMapBuilder::SharedPtr global_map_;
    IMapOptimizer::SharedPtr map_optimizer_;
    ILoopClosureDetector::SharedPtr loop_closure_detector_;

    std::optional<Eigen::Isometry3d> pending_correction_;
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> pending_corrected_poses_;

    Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();

    mutable std::mutex loop_edges_mutex_;
    std::vector<LoopEdge> loop_edges_;
};

}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__BACKEND_HPP_