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
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    return imu_preintegrator_->getLatestNavState();
}

void FrontEnd::updateMapReferenceShift(const Eigen::Isometry3d& shift_matrix) {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    map_reference_shift_transform_ = shift_matrix;
}

bool FrontEnd::SensorDataSynced() { return data_manager_->hasSyncedData(); }

std::optional<LidarFrame::SharedPtr> FrontEnd::processPipeline() {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);

    LidarData lidar;
    std::vector<ImuData> opt_imu_batch;
    if (!data_manager_->getSyncedData(lidar, opt_imu_batch)) {
        return std::nullopt;
    }

    auto processed_cloud = scan_preprocessor_->processCloud(opt_imu_batch, lidar);
    auto features = feature_extractor_->extract(processed_cloud);

    auto raw_state = imu_preintegrator_->getLatestNavState();
    auto corrected_state = raw_state;
    corrected_state.pose = map_reference_shift_transform_ * raw_state.pose;
    auto matched_result_in_map_ref = scan_matcher_->match(features, corrected_state);
    auto matched_result_in_local = matched_result_in_map_ref;
    matched_result_in_local.pose =
        map_reference_shift_transform_.inverse() * matched_result_in_map_ref.pose;

    std::vector<ImuData> reprop_imu_batch;
    data_manager_->getBatchImuData(lidar.timestamp, last_processed_imu_time_, reprop_imu_batch);
    imu_preintegrator_->updateBiasAndRepropagateImus(matched_result_in_local, opt_imu_batch,
                                                     reprop_imu_batch);

    return LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                  processed_cloud.cloud, features, matched_result_in_map_ref);
}

}  // namespace lio_slam_shaw::core