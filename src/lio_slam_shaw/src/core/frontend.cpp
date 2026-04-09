#pragma once

#include "lio_slam_shaw/core/frontend.hpp"

namespace lio_slam_shaw::core {

FrontEnd::FrontEnd(SensorDataManager::SharedPtr data_manager,
                   IScanPreprocessor::SharedPtr scan_preprocessor,
                   IFeatureExtractor::SharedPtr feature_extractor,
                   IScanMatcher::SharedPtr scan_matcher,
                   IImuPreintegrator::SharedPtr imu_preintegrator)
    : data_manager_(std::move(data_manager)),
      scan_preprocessor_(std::move(scan_preprocessor)),
      feature_extractor_(std::move(feature_extractor)),
      scan_matcher_(std::move(scan_matcher)),
      imu_preintegrator_(std::move(imu_preintegrator)) {}

void FrontEnd::feed_lidar(const LidarData& lidar) { data_manager_->addLidarData(lidar); }

void FrontEnd::feed_imu(const ImuData& imu) {
    data_manager_->addImuData(imu);

    std::vector<ImuData> imu_batch;
    if (!data_manager_->getBatchImuData(last_processed_imu_time_, imu.timestamp, imu_batch)) {
        return;
    }
    imu_preintegrator_->integrateImusAndPredict(imu_batch);
    last_processed_imu_time_ = imu.timestamp;
}

NavState FrontEnd::getLatestNavState() const {
    auto state = imu_preintegrator_->getLatestNavState();
    std::lock_guard<std::mutex> lock(correction_mtx_);
    state.pose = correction_ * state.pose;
    return state;
}

void FrontEnd::updateGlobalPose(const Eigen::Isometry3d& original_pose,
                                const Eigen::Isometry3d& corrected_pose) {
    std::lock_guard<std::mutex> lock(correction_mtx_);
    correction_ = corrected_pose * original_pose.inverse();
    correction_pending_ = true;
}

bool FrontEnd::SensorDataSynced() { return data_manager_->hasSyncedData(); }

std::optional<LidarFrame::SharedPtr> FrontEnd::processPipeline() {
    LidarData lidar;
    std::vector<ImuData> opt_imu_batch;

    if (!data_manager_->getSyncedData(lidar, opt_imu_batch)) {
        return std::nullopt;
    }

    // 整個 pipeline 使用同一份 snapshot，避免後端非同步修改造成不一致
    Eigen::Isometry3d correction_snapshot;
    bool pending;
    {
        std::lock_guard<std::mutex> lock(correction_mtx_);
        correction_snapshot = correction_;
        pending = correction_pending_;
    }

    auto processed_cloud = scan_preprocessor_->processCloud(opt_imu_batch, lidar);
    auto features = feature_extractor_->extract(processed_cloud);

    // scan matcher 用 corrected frame 作為初始猜測
    auto raw_state = imu_preintegrator_->getLatestNavState();
    auto corrected_state = raw_state;
    corrected_state.pose = correction_snapshot * raw_state.pose;
    auto matched_result = scan_matcher_->match(features, corrected_state);

    // 直接傳入 preintegrator：
    // - 正常情況: correction_ = Identity → matched_result 即為 raw frame，無需轉換
    // - pending 情況: matched_result 在 corrected frame，當作新 anchor 傳入
    //   GTSAM graph 以此重建基點，之後 correction_ 歸 Identity
    std::vector<ImuData> reprop_imu_batch;
    data_manager_->getBatchImuData(lidar.timestamp, last_processed_imu_time_, reprop_imu_batch);
    imu_preintegrator_->updateBiasAndRepropagateImus(matched_result, opt_imu_batch,
                                                     reprop_imu_batch);

    if (pending) {
        std::lock_guard<std::mutex> lock(correction_mtx_);
        correction_ = Eigen::Isometry3d::Identity();
        correction_pending_ = false;
    }

    return LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                  processed_cloud.cloud, features, matched_result);
}

}  // namespace lio_slam_shaw::core