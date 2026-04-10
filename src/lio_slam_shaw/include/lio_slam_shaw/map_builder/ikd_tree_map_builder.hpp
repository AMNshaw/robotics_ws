#ifndef LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_

#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree.h"

namespace lio_slam_shaw::map_builder {

struct IkdTreeMapBuilderParams {
    // keyframe 判斷條件（與上一個 keyframe 的距離/角度閾值）
    double keyframe_distance_threshold = 1.0;  // meters
    double keyframe_angle_threshold = 0.2;     // radians

    // ikd-Tree 參數
    float ikd_delete_param = 0.5f;
    float ikd_balance_param = 0.6f;
    float ikd_downsample_size = 0.3f;  // voxel size for downsampling on insert

    // queryNearestPoints 最大搜尋距離
    double max_search_dist = 5.0;

    // 平面擬合最小點數（法向量有效性判斷）
    int min_plane_points = 5;
    // 平面有效性（最小/最大特徵值比值）
    double plane_valid_threshold = 0.1;
};

class IkdTreeMapBuilder : public core::IMapBuilder {
public:
    explicit IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params = {});
    ~IkdTreeMapBuilder() override = default;

    std::optional<core::KeyFrame::SharedPtr> addFrame(
        const core::LidarFrame::SharedPtr& frame) override;

    std::vector<core::NearestPointResult> queryNearestPoints(
        const core::PointCloudIRTPtr& query_cloud, const Eigen::Isometry3d& T_map_lidar,
        int k = 5) const override;

    void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) override;

    std::vector<core::KeyFrame::SharedPtr> getAllKeyframes() const override;
    std::optional<core::KeyFrame::SharedPtr> getKeyframe(uint64_t id) const override;
    std::optional<core::KeyFrame::SharedPtr> getLatestKeyframe() const override;
    core::PointCloudIRTPtr getGlobalMap() const override;

private:
    // 判斷是否要建立新 keyframe（給後端 pose graph 用，與 ikd-Tree 插入無關）
    bool isNewKeyframe(const Eigen::Isometry3d& pose) const;

    // 平面擬合（PCA），回傳法向量與重心
    core::NearestPointResult fitPlane(const KD_TREE<core::PointXYZIRT>::PointVector& neighbors,
                                      const Eigen::Vector3d& query_point_in_map) const;

    IkdTreeMapBuilderParams params_;
    KD_TREE<core::PointXYZIRT> ikd_tree_;

    std::vector<core::KeyFrame::SharedPtr> keyframes_;
    std::unordered_map<uint64_t, size_t> keyframe_index_;  // id → keyframes_ 的 index

    uint64_t next_keyframe_id_ = 0;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
