#ifndef LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct Keyframe {
    using SharedPtr = std::shared_ptr<Keyframe>;

    uint64_t id;
    Timestamp timestamp;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 全局地圖座標系
    FeatureSet features;
    PointCloudIRTConstPtr cloud_body;
    core::ScanMatchResult matched_result;

    Keyframe(uint64_t id, const Timestamp& timestamp, const Eigen::Isometry3d& pose,
             const FeatureSet& features, PointCloudIRTConstPtr cloud_body,
             const core::ScanMatchResult& matched_result)
        : id(id),
          timestamp(timestamp),
          pose(pose),
          features(features),
          cloud_body(cloud_body),
          matched_result(matched_result) {}
};

class IMapBuilder {
public:
    using SharedPtr = std::shared_ptr<IMapBuilder>;
    using ConstSharedPtr = std::shared_ptr<const IMapBuilder>;

    virtual ~IMapBuilder() = default;

    virtual std::optional<Keyframe::SharedPtr> addFrame(const LidarFrame::SharedPtr& frame) = 0;
    virtual void addKeyFrame(const Keyframe::SharedPtr& keyframe) = 0;

    virtual void clearMap() = 0;

    /// Returns true once the underlying map has at least one frame ingested.  Callers (e.g. iEKF
    /// odometry) use this to skip scan-to-map updates on the very first frame, when KNN queries
    /// would return garbage.
    virtual bool isMapReady() const = 0;

    virtual void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) = 0;

    virtual void updateMap() = 0;

    virtual std::vector<Keyframe::SharedPtr> getAllKeyframes() const = 0;
    virtual std::optional<Keyframe::SharedPtr> getKeyframe(uint64_t id) const = 0;
    virtual std::optional<Keyframe::SharedPtr> getLatestKeyframe() const = 0;

    virtual PointCloudIRTPtr getGlobalMap() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_
