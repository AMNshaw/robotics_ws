#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

ImuDeskewPreprocessor::ImuDeskewPreprocessor(core::IImuPreintegrator::SharedPtr imu_preintegrator)
    : imu_preintegrator_(std::move(imu_preintegrator)) {}

// =============================================================================
// ImuDeskewPreprocessor::processCloud
// =============================================================================
//
// 去運動失真 (Motion Undistortion / Deskewing)
//
// 一幀 LiDAR 掃描耗時約 0.1s。在此期間機器人持續運動，
// 導致不同時刻採集的點實際對應不同位姿，形成「運動失真」。
//
// 修正方式（全 6-DoF 補償）：
//   以掃描起始時刻 t_0 的 body frame 為參考系，
//   對第 i 個點（採集時刻 t_i）：
//     T_{b_i \to b_0} = T_0^{-1} * T_i
//     p_{b_0} = T_{b_i \to b_0} * p_{b_i}
//
// 旋轉狀態由 IImuPreintegrator::queryNavState 取得：
//   - nav_state_queue_ 在每次 IMU 積分時即時更新
//   - bias 由 GTSAM 每幀修正後，queue 清空並重建（repropagation）
//   - 查詢時做 slerp 插值，精度高於從 Identity 重頭積分
//
// imu_data 參數保留（符合 IScanPreprocessor 介面），本函式不直接使用。
//
core::LidarData ImuDeskewPreprocessor::processCloud(const std::vector<core::ImuData>& /*imu_data*/,
                                                    const core::LidarData& raw_cloud) {
    // -------------------------------------------------------------------------
    // 0. 短路
    // -------------------------------------------------------------------------
    if (!raw_cloud.cloud || raw_cloud.cloud->empty()) {
        return raw_cloud;
    }

    // -------------------------------------------------------------------------
    // 1. 查詢掃描起始時刻的 NavState，取得補償基準 T(t_0)^{-1}
    // -------------------------------------------------------------------------
    auto state_0_opt = imu_preintegrator_->queryNavState(raw_cloud.time_start);
    if (!state_0_opt.has_value()) {
        return raw_cloud;  // queue 為空（尚未初始化），跳過補償
    }
    const Eigen::Isometry3d T_0_inv = state_0_opt->pose.inverse();

    // -------------------------------------------------------------------------
    // 2. 對每個點查詢插值狀態，計算 T_rel 並補償
    // -------------------------------------------------------------------------
    auto deskewed = std::make_shared<core::PointCloudIRT>();
    deskewed->reserve(raw_cloud.cloud->size());

    for (const auto& pt : raw_cloud.cloud->points) {
        auto pt_time =
            raw_cloud.time_start + std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       std::chrono::duration<double>(static_cast<double>(pt.time)));

        auto state_i_opt = imu_preintegrator_->queryNavState(pt_time);
        if (!state_i_opt.has_value()) {
            deskewed->push_back(pt);
            continue;
        }
        const auto T_i = state_i_opt->pose;
        const Eigen::Isometry3d T_ti_to_t0 = T_0_inv * T_i;

        const Eigen::Vector3f p_orig(pt.x, pt.y, pt.z);
        const Eigen::Vector3f p_deskewed =
            (T_ti_to_t0 * p_orig.cast<double>()).cast<float>().eval();

        core::PointXYZIRT new_pt = pt;
        new_pt.x = p_deskewed.x();
        new_pt.y = p_deskewed.y();
        new_pt.z = p_deskewed.z();
        deskewed->push_back(new_pt);
    }

    core::LidarData result = raw_cloud;
    result.cloud = deskewed;
    return result;
}

}  // namespace lio_slam_shaw::lidar_preprocessor