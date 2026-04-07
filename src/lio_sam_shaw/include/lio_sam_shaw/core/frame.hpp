#ifndef LIO_SAM_SHAW__CORE__FRAME_HPP_
#define LIO_SAM_SHAW__CORE__FRAME_HPP_

#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

/**
 * @brief LidarFrame 核心資料單元
 */
struct LidarFrame {
    using Ptr = std::shared_ptr<LidarFrame>;

    uint64_t id;
    double timestamp;

    // 點雲數據指標
    pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr deskewed_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr edge_features;
    pcl::PointCloud<PointXYZIRT>::Ptr surf_features;

    // 狀態量
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    ImuBias bias;

    bool is_keyframe = false;

    // 建構子
    LidarFrame(uint64_t frame_id, double stamp)
        : id(frame_id),
          timestamp(stamp),
          raw_cloud(new pcl::PointCloud<PointXYZIRT>()),
          deskewed_cloud(new pcl::PointCloud<PointXYZIRT>()),
          edge_features(new pcl::PointCloud<PointXYZIRT>()),
          surf_features(new pcl::PointCloud<PointXYZIRT>()) {}

    /**
     * @brief 靜態工廠函式，統一管理 Frame 的生成
     */
    static Ptr make_frame(uint64_t id, double stamp) {
        return std::make_shared<LidarFrame>(id, stamp);
    }
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__FRAME_HPP_