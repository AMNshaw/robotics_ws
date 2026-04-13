#ifndef LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct KeyFrame {
    using SharedPtr = std::shared_ptr<KeyFrame>;

    uint64_t id;
    Timestamp timestamp;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 全局地圖座標系
    FeatureSet features;
    PointCloudIRTPtr cloud;
    core::ScanMatchResult matched_result;
};

// queryNearestPoints 的單點查詢結果，供 scan matcher 建構 point-to-plane 殘差使用
struct NearestPointResult {
    bool valid = false;            // 是否找到足夠鄰近點且平面擬合條件良好
    Eigen::Vector3d point_in_map;  // query 點在地圖座標系下的位置
    Eigen::Vector3d normal;        // 擬合平面法向量（指向外）
    Eigen::Vector3d centroid;      // k 個鄰近點的重心（用於計算點到面距離）
};

class IMapBuilder {
public:
    using SharedPtr = std::shared_ptr<IMapBuilder>;
    using ConstSharedPtr = std::shared_ptr<const IMapBuilder>;

    virtual ~IMapBuilder() = default;

    virtual std::optional<KeyFrame::SharedPtr> addFrame(const LidarFrame::SharedPtr& frame) = 0;
    virtual void addKeyFrame(const KeyFrame::SharedPtr& keyframe) = 0;

    virtual void clearMap() = 0;

    virtual std::vector<NearestPointResult> queryNearestPoints(const PointCloudIRTPtr& query_cloud,
                                                               const Eigen::Isometry3d& T_map_lidar,
                                                               int k = 5) const = 0;

    virtual void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) = 0;

    virtual void updateMap() = 0;

    // 供 loop closure detector 查詢歷史 keyframe
    virtual std::vector<KeyFrame::SharedPtr> getAllKeyframes() const = 0;
    virtual std::optional<KeyFrame::SharedPtr> getKeyframe(uint64_t id) const = 0;
    virtual std::optional<KeyFrame::SharedPtr> getLatestKeyframe() const = 0;

    // 用於可視化全局地圖
    virtual PointCloudIRTPtr getGlobalMap() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_MAP_BUILDER_HPP_
