#ifndef LIO_SLAM_SHAW__CORE__FRONTEND_HPP_
#define LIO_SLAM_SHAW__CORE__FRONTEND_HPP_

#include <chrono>
#include <mutex>

#include "lio_slam_shaw/core/i_feature_extractor.hpp"
#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/core/lidar_frame.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class FrontEnd {
public:
    using SharedPtr = std::shared_ptr<FrontEnd>;
    using ConstSharedPtr = std::shared_ptr<const FrontEnd>;

    // clang-format off
    FrontEnd(SensorDataManager::SharedPtr data_manager,
             IScanPreprocessor::SharedPtr scan_preprocessor,
             IFeatureExtractor::SharedPtr feature_extractor, 
             IScanMatcher::SharedPtr scan_matcher,
             IImuPreintegrator::SharedPtr imu_preintegrator);
    // clang-format on
    ~FrontEnd() = default;

    void feed_lidar(const LidarData& lidar);
    void feed_imu(const ImuData& imu);

    NavState getLatestOdomState() const;
    bool SensorDataSynced();

    std::optional<LidarFrame::SharedPtr> processPipeline();

    void setOdomToMapTransform(const Eigen::Isometry3d& T_map_odom);

private:
    mutable std::mutex pipeline_mtx_;
    SensorDataManager::SharedPtr data_manager_;
    IScanPreprocessor::SharedPtr scan_preprocessor_;
    IFeatureExtractor::SharedPtr feature_extractor_;
    IScanMatcher::SharedPtr scan_matcher_;
    IImuPreintegrator::SharedPtr imu_preintegrator_;

    Timestamp last_processed_imu_time_;
    uint64_t frame_id_counter_ = 0;

    Eigen::Isometry3d T_map_odom_ = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> latest_scan_match_cov_ = Eigen::Matrix<double, 6, 6>::Identity();
};
}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__FRONTEND_HPP_
