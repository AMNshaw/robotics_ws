#ifndef LIO_SAM_SHAW__CORE__I_MAP_BUILDER_HPP_
#define LIO_SAM_SHAW__CORE__I_MAP_BUILDER_HPP_

#include <memory>
#include <optional>
#include <vector>

#include "lio_sam_shaw/core/lidar_frame.hpp"
#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

// 儲存於 MapBuilder 中的 keyframe 資料
struct KeyFrame {
    using SharedPtr = std::shared_ptr<KeyFrame>;

    uint64_t id;
    Timestamp timestamp;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 全局地圖座標系
    FeatureSet features;
    PointCloudIRTPtr cloud;  // deskewed 點雲
};

class IMapBuilder {
public:
    using SharedPtr = std::shared_ptr<IMapBuilder>;
    using ConstSharedPtr = std::shared_ptr<const IMapBuilder>;

    virtual ~IMapBuilder() = default;

    // 加入新的 frame，內部判斷是否達到 keyframe 條件
    // 是 keyframe 才加入並回傳；否則回傳 nullopt
    virtual std::optional<KeyFrame::SharedPtr> addKeyframe(const LidarFrame::SharedPtr& frame) = 0;

    // 建構以指定 keyframe 為中心的 local map，給前端 scan match 用
    virtual PointCloudIRTPtr buildLocalMap(uint64_t keyframe_id) const = 0;

    // 後端全局優化完後，批次更新所有歷史 keyframe 的 pose
    virtual void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) = 0;

    // 供 loop closure detector 查詢歷史 keyframe
    virtual std::vector<KeyFrame::SharedPtr> getKeyframes() const = 0;
    virtual std::optional<KeyFrame::SharedPtr> getKeyframe(uint64_t id) const = 0;

    // 用於可視化全局地圖
    virtual PointCloudIRTPtr getGlobalMap() const = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_MAP_BUILDER_HPP_
