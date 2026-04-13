#pragma once

#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/sensor_data.hpp"

namespace lio_slam_shaw::core {

/**
 * @brief LidarFrame
 */
struct LidarFrame {
    using Ptr = std::shared_ptr<LidarFrame>;

    uint64_t id;
    double timestamp;

    pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr deskewed_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr edge_features;
    pcl::PointCloud<PointXYZIRT>::Ptr surf_features;

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    ImuBias bias;

    bool is_keyframe = false;

    LidarFrame(uint64_t frame_id, double stamp)
        : id(frame_id),
          timestamp(stamp),
          raw_cloud(new pcl::PointCloud<PointXYZIRT>()),
          deskewed_cloud(new pcl::PointCloud<PointXYZIRT>()),
          edge_features(new pcl::PointCloud<PointXYZIRT>()),
          surf_features(new pcl::PointCloud<PointXYZIRT>()) {}

    /**
     * @brief 靜態工廠函式
     * 用法: auto frame = LidarFrame::make_frame(id, stamp);
     */
    static Ptr make_frame(uint64_t id, double stamp) {
        return std::make_shared<LidarFrame>(id, stamp);
    }
};

}  // namespace lio_slam_shaw::core