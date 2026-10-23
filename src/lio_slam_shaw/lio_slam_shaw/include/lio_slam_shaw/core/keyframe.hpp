#ifndef LIO_SLAM_SHAW__CORE__KEYFRAME_HPP_
#define LIO_SLAM_SHAW__CORE__KEYFRAME_HPP_

#include <memory>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct Keyframe {
    using SharedPtr = std::shared_ptr<Keyframe>;

    uint64_t id;
    Timestamp timestamp;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // 全局地圖座標系
    FeatureSet features;
    PointCloudIRTConstPtr cloud_body;
    ScanMatchResult matched_result;

    Keyframe(uint64_t id, const Timestamp& timestamp, const Eigen::Isometry3d& pose,
             const FeatureSet& features, PointCloudIRTConstPtr cloud_body,
             const ScanMatchResult& matched_result)
        : id(id),
          timestamp(timestamp),
          pose(pose),
          features(features),
          cloud_body(std::move(cloud_body)),
          matched_result(matched_result) {}
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__KEYFRAME_HPP_
