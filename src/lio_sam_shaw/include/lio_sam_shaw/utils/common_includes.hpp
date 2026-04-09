#pragma once

#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/sensor_data.hpp"

namespace lio_slam_shaw::core {

/**
 * @brief LidarFrame 是系統中傳遞的核心資料單元
 * 包含了點雲、特徵、以及對應的位姿狀態
 */
struct LidarFrame {
    using Ptr = std::shared_ptr<LidarFrame>;

    // --- 基礎資訊 ---
    uint64_t id;
    double timestamp;

    // --- 點雲資料 (使用 PCL 指標) ---
    pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud;       // 原始未處理點雲
    pcl::PointCloud<PointXYZIRT>::Ptr deskewed_cloud;  // 去畸變後的點雲
    pcl::PointCloud<PointXYZIRT>::Ptr edge_features;   // 提取出的邊緣特徵
    pcl::PointCloud<PointXYZIRT>::Ptr surf_features;   // 提取出的平面特徵

    // --- 狀態估計 (會被 Optimizer 更新) ---
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 世界坐標系下的位姿
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();           // 世界坐標系下的速度
    ImuBias bias;                                            // 該影格對應的 IMU 零偏

    // --- 旗標與屬性 ---
    bool is_keyframe = false;

    // --- 建構子 (建議設為 Public 以利於 make_shared) ---
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