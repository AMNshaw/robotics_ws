#ifndef LIO_SLAM_SHAW__CORE__FRONTEND_HPP_
#define LIO_SLAM_SHAW__CORE__FRONTEND_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

#include "lio_slam_shaw/core/i_feature_extractor.hpp"
#include "lio_slam_shaw/core/i_lio_initializer.hpp"
#include "lio_slam_shaw/core/i_local_map_builder.hpp"
#include "lio_slam_shaw/core/i_odometry_estimator.hpp"
#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct FrontEndParams {
    // 0 disables dropping. Dropping LiDAR frames is useful for live real-time
    // demos, but it changes the scan-to-map sequence and can create large drift
    // during offline dataset evaluation.
    size_t max_pending_lidar_queue = 0;
};

class FrontEnd {
public:
    using SharedPtr = std::shared_ptr<FrontEnd>;
    using ConstSharedPtr = std::shared_ptr<const FrontEnd>;

    FrontEnd(IScanPreprocessor::SharedPtr scan_preprocessor,
             IFeatureExtractor::SharedPtr feature_extractor,
             IOdometryEstimator::SharedPtr odometry_estimator,
             ILocalMapBuilder::SharedPtr local_map,
             ILioInitializer::SharedPtr initializer = nullptr, const FrontEndParams& params = {});
    ~FrontEnd();

    void feed_lidar(const LidarData& lidar);
    void feed_imu(const ImuData& imu);

    NavState getLatestOdomState() const;
    bool SensorDataSynced();

    std::optional<LidarFrame::SharedPtr> processPipeline();

private:
    mutable std::mutex pipeline_mtx_;
    IScanPreprocessor::SharedPtr scan_preprocessor_;
    IFeatureExtractor::SharedPtr feature_extractor_;
    IOdometryEstimator::SharedPtr odometry_estimator_;
    ILocalMapBuilder::SharedPtr local_map_;
    ILioInitializer::SharedPtr initializer_;
    FrontEndParams params_;
    std::atomic<bool> initialized_{false};

    // Pending lidar queue (written by feed_lidar, consumed by processPipeline)
    std::deque<LidarData> pending_lidar_;
    mutable std::mutex lidar_mutex_;
    Timestamp last_imu_time_{};

    // Initialisation worker thread
    void initThread();
    /// Estimate gravity from mean IMU accel — always succeeds, zero velocity/bias.
    LioInitResult staticImuFallback() const;
    std::thread init_thread_;
    std::mutex init_cv_mutex_;
    std::condition_variable init_cv_;

    uint64_t frame_id_counter_ = 0;
};
}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__FRONTEND_HPP_
