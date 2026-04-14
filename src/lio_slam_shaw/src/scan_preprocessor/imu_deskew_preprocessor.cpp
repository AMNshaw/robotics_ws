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
    result.vel = s0.vel + alpha * (s1.vel - s0.vel);
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

    auto state_0 = interpolateNavState(snapshot, raw_cloud.time_start);
    if (!state_0.has_value()) return raw_cloud;

    const Eigen::Isometry3d T_0_inv = state_0->pose.inverse();
    const int n = static_cast<int>(raw_cloud.cloud->size());

    auto deskewed = std::make_shared<core::PointCloudIRT>();
    deskewed->resize(n);

#pragma omp parallel for schedule(dynamic, 64) default(none) \
    shared(snapshot, raw_cloud, deskewed, T_0_inv, n)
    for (int i = 0; i < n; ++i) {
        const auto& pt = raw_cloud.cloud->points[i];
        const auto pt_time =
            raw_cloud.time_start + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::duration<double>(static_cast<double>(pt.time)));

        auto state_i = interpolateNavState(snapshot, pt_time);
        if (!state_i.has_value()) {
            (*deskewed)[i] = pt;
            continue;
        }

        const Eigen::Isometry3d T_ti_to_t0 = T_0_inv * state_i->pose;
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

    if (params_.voxel_leaf_size > 0.0f) {
        pcl::VoxelGrid<core::PointXYZIRT> voxel;
        voxel.setLeafSize(params_.voxel_leaf_size, params_.voxel_leaf_size,
                          params_.voxel_leaf_size);
        voxel.setInputCloud(result.cloud);
        auto filtered = std::make_shared<core::PointCloudIRT>();
        voxel.filter(*filtered);
        result.cloud = filtered;
    }

    return result;
}

}  // namespace lio_slam_shaw::lidar_preprocessor
