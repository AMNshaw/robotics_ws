#include "lio_slam_shaw/core/frontend.hpp"

namespace lio_slam_shaw::core {

FrontEnd::FrontEnd(SensorDataManager::SharedPtr data_manager,
                   IScanPreprocessor::SharedPtr scan_preprocessor,
                   IFeatureExtractor::SharedPtr feature_extractor,
                   IOdometryEstimator::SharedPtr odometry_estimator)
    : data_manager_(std::move(data_manager)),
      scan_preprocessor_(std::move(scan_preprocessor)),
      feature_extractor_(std::move(feature_extractor)),
      odometry_estimator_(std::move(odometry_estimator)) {}

void FrontEnd::feed_lidar(const LidarData& lidar) { data_manager_->addLidarData(lidar); }

void FrontEnd::feed_imu(const ImuData& imu) {
    data_manager_->addImuData(imu);
    odometry_estimator_->feedImu(imu);
}

NavState FrontEnd::getLatestOdomState() const { return odometry_estimator_->getLatestState(); }

void FrontEnd::setOdomToMapTransform(const Eigen::Isometry3d& T_map_odom) {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    odometry_estimator_->setMapToOdomTransform(T_map_odom);
}

bool FrontEnd::SensorDataSynced() { return data_manager_->hasSyncedData(); }

std::optional<LidarFrame::SharedPtr> FrontEnd::processPipeline() {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);

    LidarData lidar;
    std::vector<ImuData> opt_imu_batch;  // consumed by getSyncedData but not used here

    if (!data_manager_->getSyncedData(lidar, opt_imu_batch)) {
        return std::nullopt;
    }

    // 1. Deskew using predicted nav states from the odometry estimator
    auto nav_snapshot = odometry_estimator_->getNavStateQueueSnapshot();
    auto processed_cloud = scan_preprocessor_->processCloud(nav_snapshot, lidar);

    // 2. Feature extraction
    auto features = feature_extractor_->extract(processed_cloud);

    // 3. Odometry estimation (iEKF update + IMU repropagate — all internal)
    auto odom_result = odometry_estimator_->estimateWithFeatures(features, lidar.timestamp);

    if (!odom_result.matched_in_map.is_converged) {
        return std::nullopt;
    }

    // 4. Build LidarFrame
    auto frame = LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                        processed_cloud.cloud, features, odom_result.matched_in_map,
                                        odom_result.corrected_state);
    return frame;
}

}  // namespace lio_slam_shaw::core