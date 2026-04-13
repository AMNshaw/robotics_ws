#ifndef LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_

#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree.h"

namespace lio_slam_shaw::map_builder {

struct IkdTreeMapBuilderParams {
    double keyframe_distance_threshold = 1.0;
    double keyframe_angle_threshold = 0.2;

    float ikd_delete_param = 0.5f;
    float ikd_balance_param = 0.6f;
    float ikd_downsample_size = 0.3f;

    double max_search_dist = 5.0;

    int min_plane_points = 5;
    double plane_valid_threshold = 0.1;
};

class IkdTreeMapBuilder : public core::IMapBuilder {
public:
    explicit IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params = {});
    ~IkdTreeMapBuilder() override = default;

    std::optional<core::KeyFrame::SharedPtr> addFrame(
        const core::LidarFrame::SharedPtr& frame) override;
    void addKeyFrame(const core::KeyFrame::SharedPtr& keyframe) override;

    void clearMap() override;

    std::vector<core::NearestPointResult> queryNearestPoints(
        const core::PointCloudIRTPtr& query_cloud, const Eigen::Isometry3d& T_map_lidar,
        int k = 5) const override;

    void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) override;

    void updateMap() override;

    std::vector<core::KeyFrame::SharedPtr> getAllKeyframes() const override;
    std::optional<core::KeyFrame::SharedPtr> getKeyframe(uint64_t id) const override;
    std::optional<core::KeyFrame::SharedPtr> getLatestKeyframe() const override;
    core::PointCloudIRTPtr getGlobalMap() const override;

private:
    bool isNewKeyframe(const Eigen::Isometry3d& pose) const;

    core::NearestPointResult fitPlane(const KD_TREE<core::PointXYZIRT>::PointVector& neighbors,
                                      const Eigen::Vector3d& query_point_in_map) const;

    IkdTreeMapBuilderParams params_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> ikd_tree_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> temp_ikd_tree_;

    std::atomic<bool> is_updating_map_{false};
    mutable std::mutex increment_mutex_;
    KD_TREE<core::PointXYZIRT>::PointVector ikd_increment_;
    mutable std::shared_mutex correction_mutex_;
    std::optional<Eigen::Isometry3d> last_pending_correction_;

    mutable std::shared_mutex keyframe_mutex_;
    std::vector<core::KeyFrame::SharedPtr> keyframes_;
    std::unordered_map<uint64_t, size_t> keyframe_index_;  // id → keyframes_ 的 index

    std::atomic<uint64_t> next_keyframe_id_{0};
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
