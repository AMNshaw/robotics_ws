#ifndef LIO_SLAM_SHAW__CORE__SENSOR_DATA_TYPES_HPP_
#define LIO_SLAM_SHAW__CORE__SENSOR_DATA_TYPES_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <chrono>

namespace lio_slam_shaw::core {

using Timestamp = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>;
using Duration = std::chrono::nanoseconds;

inline double toSeconds(const Duration& d) { return std::chrono::duration<double>(d).count(); }

inline double getDeltaSec(const Timestamp& t_start, const Timestamp& t_end) {
    std::chrono::duration<double> diff = t_end - t_start;
    return diff.count();
}

struct NavState {
    Timestamp timestamp;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> pose_cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;
    Eigen::Vector3d acc_bias = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyr_bias = Eigen::Vector3d::Zero();
};

// 1. 自定義的 LIO-SAM 點雲格式
struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct LidarData {
    Timestamp timestamp;   // 這一幀的基準時間 (通常是 ROS Header 的時間)
    Timestamp time_start;  // 第一顆點的絕對時間
    Timestamp time_end;    // 最後一顆點的絕對時間

    // 2. 數據本體 (使用 Smart Pointer 實現 Zero-copy)
    PointCloudIRTPtr cloud;
    LidarData() = default;
    LidarData(Timestamp t, Timestamp t_s, Timestamp t_e, const PointCloudIRTPtr& c)
        : timestamp(t), time_start(t_s), time_end(t_e), cloud(c) {}
};

using PointCloudIRT = pcl::PointCloud<PointXYZIRT>;
using PointCloudIRTPtr = PointCloudIRT::Ptr;
using PointCloudIRTConstPtr = PointCloudIRT::ConstPtr;

struct ImuData {
    Timestamp timestamp;  // 改用 chrono
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr;
};

struct FeatureSet {
    PointCloudIRTPtr raw_deskewed;
    PointCloudIRTPtr edge;
    PointCloudIRTPtr surf;
    FeatureSet()
        : raw_deskewed(new pcl::PointCloud<PointXYZIRT>()),
          edge(new pcl::PointCloud<PointXYZIRT>()),
          surf(new pcl::PointCloud<PointXYZIRT>()) {}
};

struct ScanMatchResult {
    // 1. 匹配收斂後的 6-DoF 位姿 (通常是在 Map 座標系下的 Global Pose)
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e-4;

    // 3. 匹配健康度指標
    bool is_converged = false;   // ICP 有沒有收斂？
    bool is_degenerate = false;  // 是不是處於缺乏特徵的退化環境 (例如空曠隧道)？
    double fitness_score = 0.0;  // 點到面的平均殘差
};

}  // namespace lio_slam_shaw::core

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(lio_slam_shaw::core::PointXYZIRT,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (uint16_t, ring, ring)
    (float, time, time)
)
// clang-format on

#endif  // LIO_SLAM_SHAW__CORE__SENSOR_DATA_TYPES_HPP_