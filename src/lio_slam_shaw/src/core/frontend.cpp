#include "lio_slam_shaw/core/frontend.hpp"

#include <chrono>
#include <iostream>

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
    using Clock = std::chrono::steady_clock;
    const auto t_pipe_start = Clock::now();

    LidarData lidar;
    std::vector<ImuData> opt_imu_batch;  // consumed by getSyncedData but not used here

    if (!data_manager_->getSyncedData(lidar, opt_imu_batch)) {
        return std::nullopt;
    }
    const auto t_sync = Clock::now();

    // 1. Deskew using predicted nav states from the odometry estimator
    auto nav_snapshot = odometry_estimator_->getNavStateQueueSnapshot();
    auto processed_cloud = scan_preprocessor_->processCloud(nav_snapshot, lidar);
    const auto t_deskew = Clock::now();

    // 2. Feature extraction
    auto features = feature_extractor_->extract(processed_cloud);
    const auto t_extract = Clock::now();

    // 3. Odometry estimation (iEKF update + IMU repropagate — all internal)
    auto odom_result = odometry_estimator_->estimateWithFeatures(features, lidar.timestamp);
    const auto t_odom = Clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    std::clog << "[FrontEnd] timing(ms): sync=" << ms(t_pipe_start, t_sync)
              << " deskew=" << ms(t_sync, t_deskew) << " extract=" << ms(t_deskew, t_extract)
              << " odom=" << ms(t_extract, t_odom) << " total=" << ms(t_pipe_start, t_odom)
              << " | cloud=" << lidar.cloud->size()
              << " converged=" << odom_result.matched_in_map.is_converged << '\n';

    if (!odom_result.matched_in_map.is_converged) {
        return std::nullopt;
    }

    // 4. Build LidarFrame
    auto frame = LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                        processed_cloud.cloud, features, odom_result.matched_in_map,
                                        odom_result.corrected_state);
    std::clog << "[FrontEnd] frame #" << (frame_id_counter_ - 1) << " built OK" << '\n';
    return frame;
}

}  // namespace lio_slam_shaw::core