#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

#include <omp.h>
#include <pcl/filters/voxel_grid.h>

#include <algorithm>
#include <optional>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp>

namespace lio_slam_shaw::lidar_preprocessor {

ImuDeskewPreprocessor::ImuDeskewPreprocessor(const ImuDeskewPreprocessorParams& params)
    : params_(params) {}
namespace {

std::optional<core::NavState> interpolateNavState(const std::vector<core::NavState>& snapshot,
                                                  const core::Timestamp& t) {
    if (snapshot.empty()) return std::nullopt;
    if (t <= snapshot.front().timestamp) return snapshot.front();
    if (t >= snapshot.back().timestamp) return snapshot.back();

    auto it = std::lower_bound(
        snapshot.begin(), snapshot.end(), t,
        [](const core::NavState& s, const core::Timestamp& ts) { return s.timestamp < ts; });

    const auto& s1 = *it;
    const auto& s0 = *(it - 1);

    double total = core::getDeltaSec(s0.timestamp, s1.timestamp);
    if (total < 1e-9) return s0;
    double alpha = std::clamp(core::getDeltaSec(s0.timestamp, t) / total, 0.0, 1.0);

    Eigen::Quaterniond q0(s0.pose.linear());
    Eigen::Quaterniond q1(s1.pose.linear());

    core::NavState result;
    result.timestamp = t;
    result.pose.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
    result.pose.translation() =
        s0.pose.translation() + alpha * (s1.pose.translation() - s0.pose.translation());
    result.linear_vel = s0.linear_vel + alpha * (s1.linear_vel - s0.linear_vel);
    result.acc_bias = s0.acc_bias;
    result.gyr_bias = s0.gyr_bias;
    result.pose_cov = s0.pose_cov;
    return result;
}

}  // namespace

core::LidarData ImuDeskewPreprocessor::processCloud(const std::vector<core::NavState>& snapshot,
                                                    const core::LidarData& raw_cloud) {
    if (!raw_cloud.cloud || raw_cloud.cloud->empty()) return raw_cloud;
    if (snapshot.empty()) return raw_cloud;

    // --- 1. Deskew in LiDAR frame using body/base poses and LiDAR extrinsics ---
    auto input_cloud = raw_cloud.cloud;
    auto state_0 = interpolateNavState(snapshot, raw_cloud.time_start);
    if (!state_0.has_value()) {
        return raw_cloud;
    }

    const Eigen::Isometry3d T_world_lidar0 = state_0->pose * params_.T_base_lidar;
    const Eigen::Isometry3d T_lidar0_world = T_world_lidar0.inverse();
    const int n = static_cast<int>(input_cloud->size());

    auto deskewed = std::make_shared<core::PointCloudIRT>();
    deskewed->resize(n);

#pragma omp parallel for schedule(static)
    for (int i = 0; i < n; ++i) {
        const auto& pt = input_cloud->points[i];
        const auto pt_time =
            raw_cloud.time_start + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::duration<double>(static_cast<double>(pt.time)));

        auto state_i = interpolateNavState(snapshot, pt_time);
        if (!state_i.has_value()) {
            (*deskewed)[i] = pt;
            continue;
        }

        const Eigen::Isometry3d T_world_lidari = state_i->pose * params_.T_base_lidar;
        const Eigen::Isometry3d T_ti_to_t0 = T_lidar0_world * T_world_lidari;
        const Eigen::Vector3f p_deskewed =
            (T_ti_to_t0 * Eigen::Vector3d(pt.x, pt.y, pt.z)).cast<float>();

        core::PointXYZIRT new_pt = pt;
        new_pt.x = p_deskewed.x();
        new_pt.y = p_deskewed.y();
        new_pt.z = p_deskewed.z();
        (*deskewed)[i] = new_pt;
    }

    core::LidarData result = raw_cloud;
    result.cloud = deskewed;

    // --- 2. Downsample after deskew so voxel centroids are computed in a consistent time frame ---
    if (params_.voxel_leaf_size > 0.0f) {
        pcl::VoxelGrid<core::PointXYZIRT> voxel;
        voxel.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size,
                          params_.voxel_leaf_size);
        voxel.setInputCloud(deskewed);
        auto filtered = std::make_shared<core::PointCloudIRT>();
        voxel.filter(*filtered);
        result.cloud = filtered;
    }
    return result;
}

}  // namespace lio_slam_shaw::lidar_preprocessor
