#ifndef LIO_SLAM_SHAW__CORE__I_GLOBAL_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__CORE__I_GLOBAL_MAP_BUILDER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "lio_slam_shaw/core/keyframe.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

/// Global map builder: manages keyframes and handles loop-closure pose corrections.
class IGlobalMapBuilder {
public:
    using SharedPtr = std::shared_ptr<IGlobalMapBuilder>;
    using ConstSharedPtr = std::shared_ptr<const IGlobalMapBuilder>;

    virtual ~IGlobalMapBuilder() = default;

    /// Decide whether to create a keyframe from this frame. Returns the new
    /// keyframe if selection criteria are met, nullopt otherwise.
    virtual std::optional<Keyframe::SharedPtr> addKeyFrame(const LidarFrame::SharedPtr& frame) = 0;

    /// Update stored keyframe poses after backend optimization / loop closure.
    virtual void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) = 0;

    virtual std::vector<Keyframe::SharedPtr> getAllKeyframes() const = 0;
    virtual std::optional<Keyframe::SharedPtr> getKeyframe(uint64_t id) const = 0;
    virtual std::optional<Keyframe::SharedPtr> getLatestKeyframe() const = 0;
    virtual size_t getKeyframeCount() const = 0;

    /// Return the full global map as a point cloud (reconstructed from keyframes).
    virtual PointCloudIRTPtr getGlobalMap() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_GLOBAL_MAP_BUILDER_HPP_
