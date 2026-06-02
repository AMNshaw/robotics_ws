#ifndef LIO_SLAM_SHAW__MAP_BUILDER__KEYFRAME_GLOBAL_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__KEYFRAME_GLOBAL_MAP_BUILDER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_global_map_builder.hpp"

namespace lio_slam_shaw::map_builder {

struct KeyframeGlobalMapBuilderParams {
    double keyframe_distance_threshold = 1.0;
    double keyframe_angle_threshold = 0.2;
    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
};

/// Global map builder that manages keyframes as a flat vector + index map.
class KeyframeGlobalMapBuilder : public core::IGlobalMapBuilder {
public:
    using SharedPtr = std::shared_ptr<KeyframeGlobalMapBuilder>;

    explicit KeyframeGlobalMapBuilder(const KeyframeGlobalMapBuilderParams& params = {});
    ~KeyframeGlobalMapBuilder() override = default;

    std::optional<core::Keyframe::SharedPtr> addKeyFrame(
        const core::LidarFrame::SharedPtr& frame) override;

    void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) override;

    std::vector<core::Keyframe::SharedPtr> getAllKeyframes() const override;
    std::optional<core::Keyframe::SharedPtr> getKeyframe(uint64_t id) const override;
    std::optional<core::Keyframe::SharedPtr> getLatestKeyframe() const override;
    core::PointCloudIRTPtr getGlobalMap() const override;

private:
    bool isNewKeyframe(const Eigen::Isometry3d& pose) const;

    KeyframeGlobalMapBuilderParams params_;

    mutable std::shared_mutex keyframe_mutex_;
    std::vector<core::Keyframe::SharedPtr> keyframes_;
    std::unordered_map<uint64_t, size_t> keyframe_index_;
    std::atomic<uint64_t> next_keyframe_id_{0};
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__KEYFRAME_GLOBAL_MAP_BUILDER_HPP_
