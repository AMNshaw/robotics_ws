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

    // 回傳經全局修正後的最新狀態
    NavState getLatestNavState() const;
    bool SensorDataSynced();

    std::optional<LidarFrame::SharedPtr> processPipeline();

    // 後端 loop closure 完成後呼叫
    // original_pose: 該 keyframe 當初 scan match 的位姿 (LidarFrame::matched_result.pose)
    // corrected_pose: map optimizer 優化後的全局位姿
    void updateGlobalPose(const Eigen::Isometry3d& original_pose,
                          const Eigen::Isometry3d& corrected_pose);

private:
    SensorDataManager::SharedPtr data_manager_;
    IScanPreprocessor::SharedPtr scan_preprocessor_;
    IFeatureExtractor::SharedPtr feature_extractor_;
    IScanMatcher::SharedPtr scan_matcher_;
    IImuPreintegrator::SharedPtr imu_preintegrator_;

    Timestamp last_processed_imu_time_;
    uint64_t frame_id_counter_ = 0;

    // 後端全局優化對前端輸出的修正量
    // correction_ = corrected_pose * original_pose.inverse()
    // correction_pending_ = true 時，下一幀 pipeline 會將 matched_result
    // 直接傳給 preintegrator 重新 anchor，之後 correction_ 歸 Identity
    mutable std::mutex correction_mtx_;
    Eigen::Isometry3d correction_ = Eigen::Isometry3d::Identity();
    bool correction_pending_ = false;
};
}  // namespace lio_slam_shaw::core
#endif  // LIO_SLAM_SHAW__CORE__FRONTEND_HPP_
