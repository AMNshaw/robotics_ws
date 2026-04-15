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
      imu_preintegrator_(std::move(imu_preintegrator)) {
    T_map_odom_ = Eigen::Isometry3d::Identity();
    last_processed_imu_time_ = Timestamp::min();
}

void FrontEnd::setLidarExtrinsics(const Eigen::Isometry3d& T_base_lidar) {
    scan_matcher_->setLidarExtrinsics(T_base_lidar);
}

void FrontEnd::setImuExtrinsics(const Eigen::Isometry3d& T_base_imu) {
    imu_preintegrator_->setImuExtrinsics(T_base_imu);
}

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

NavState FrontEnd::getLatestOdomState() const {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    auto state = imu_preintegrator_->getLatestPredictState();
    state.pose_cov = latest_scan_match_cov_;
    return state;
}

void FrontEnd::setOdomToMapTransform(const Eigen::Isometry3d& T_map_odom) {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    T_map_odom_ = T_map_odom;
}

bool FrontEnd::SensorDataSynced() { return data_manager_->hasSyncedData(); }

std::optional<LidarFrame::SharedPtr> FrontEnd::processPipeline() {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);

    LidarData lidar;
    std::vector<ImuData> opt_imu_batch;
    if (!data_manager_->getSyncedData(lidar, opt_imu_batch)) {
        return std::nullopt;
    }

    auto nav_snapshot = imu_preintegrator_->getNavStateQueueSnapshot();
    auto processed_cloud = scan_preprocessor_->processCloud(nav_snapshot, lidar);
    auto features = feature_extractor_->extract(processed_cloud);

    auto state_odom = imu_preintegrator_->getLatestPredictState();
    auto state_map = state_odom;
    // Transform the initial guess from the odom frame to the map frame.
    // This is required because our scan matching (Scan-to-Map) aligns the current
    // features against the global/local map to eliminate accumulated drift.
    state_map.pose = T_map_odom_ * state_odom.pose;
    auto matched_result_in_map = scan_matcher_->match(features, state_map);
    auto matched_result_in_odom = matched_result_in_map;
    matched_result_in_odom.pose = T_map_odom_.inverse() * matched_result_in_map.pose;

    std::vector<ImuData> reprop_imu_batch;
    data_manager_->getBatchImuData(lidar.timestamp, last_processed_imu_time_, reprop_imu_batch);

    // IMU preintegration correction must remain in the odom frame.
    // Applying map-frame feedback (like loop closures) introduces pose jumps and discontinuities,
    // which leads to large biases and potential preintegrator divergence.
    imu_preintegrator_->updateBiasAndRepropagateImus(matched_result_in_odom, opt_imu_batch,
                                                     reprop_imu_batch);

    NavState corrected_state = state_odom;
    corrected_state.timestamp = lidar.timestamp;
    corrected_state.pose = matched_result_in_odom.pose;

    auto frame = LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                        processed_cloud.cloud, features, matched_result_in_map,
                                        corrected_state);
    return frame;
}

}  // namespace lio_slam_shaw::core