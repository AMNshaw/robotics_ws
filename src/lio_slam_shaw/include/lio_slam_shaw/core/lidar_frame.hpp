#ifndef LIO_SLAM_SHAW__CORE__LIDAR_FRAME_HPP_
#define LIO_SLAM_SHAW__CORE__LIDAR_FRAME_HPP_

#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct LidarFrame {
    using SharedPtr = std::shared_ptr<LidarFrame>;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    uint64_t id;
    Timestamp timestamp;

    pcl::PointCloud<PointXYZIRT>::Ptr raw_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr deskewed_cloud;
    FeatureSet features;

    ScanMatchResult matched_result;
    NavState state_odom;

    bool is_keyframe = false;

    LidarFrame(uint64_t frame_id, Timestamp stamp)
        : id(frame_id),
          timestamp(stamp),
          raw_cloud(new pcl::PointCloud<PointXYZIRT>()),
          deskewed_cloud(new pcl::PointCloud<PointXYZIRT>()) {}

    LidarFrame(uint64_t frame_id, Timestamp stamp, const pcl::PointCloud<PointXYZIRT>::Ptr& raw,
               const pcl::PointCloud<PointXYZIRT>::Ptr& deskewed, const FeatureSet& features,
               const ScanMatchResult& matched_result, const NavState& corrected_state)
        : id(frame_id),
          timestamp(stamp),
          raw_cloud(raw),
          deskewed_cloud(deskewed),
          features(features),
          matched_result(matched_result),
          state_odom(corrected_state) {}

    static SharedPtr make_frame(uint64_t id, Timestamp stamp) {
        return std::make_shared<LidarFrame>(id, stamp);
    }

    static SharedPtr make_frame(uint64_t id, Timestamp stamp,
                                const pcl::PointCloud<PointXYZIRT>::Ptr& raw,
                                const pcl::PointCloud<PointXYZIRT>::Ptr& deskewed,
                                const FeatureSet& features, const ScanMatchResult& matched_result,
                                const NavState& corrected_state) {
        return std::make_shared<LidarFrame>(id, stamp, raw, deskewed, features, matched_result,
                                            corrected_state);
    }
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__LIDAR_FRAME_HPP_