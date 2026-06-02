#ifndef LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
#define LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <thread>

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct VisualizationData {
    Timestamp timestamp;
    Eigen::Isometry3d pose_odom;
    Eigen::Isometry3d T_map_odom;
    PointCloudIRTConstPtr scan;
};

class SlamProcessor {
public:
    using SharedPtr = std::shared_ptr<SlamProcessor>;
    using ConstSharedPtr = std::shared_ptr<const SlamProcessor>;

    using OdometryCallback = std::function<void(const NavState& odom_state)>;
    using VisualizationCallback = std::function<void(const VisualizationData& viz_data)>;

    explicit SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend);
    ~SlamProcessor();
    SlamProcessor(const SlamProcessor&) = delete;
    SlamProcessor& operator=(const SlamProcessor&) = delete;

    void start();

    void feedLidar(const LidarData& lidar);
    void feedImu(const ImuData& imu);

    void registerOdometryCallback(const OdometryCallback& callback);
    void registerVisualizationCallback(const VisualizationCallback& callback);

private:
    void frontendThread();
    void backendThread();
    void visualizationThread();

    FrontEnd::SharedPtr front_end_;
    BackEnd::SharedPtr back_end_;

    std::atomic<bool> run_{false};
    std::thread frontend_thread_;
    std::thread backend_thread_;
    std::thread visualization_thread_;

    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;

    std::mutex backend_mutex_;
    std::condition_variable backend_cv_;

    std::queue<Keyframe::SharedPtr> keyframe_queue_;
    std::shared_mutex map_mutex_;

    std::mutex viz_mutex_;
    std::condition_variable viz_cv_;
    std::deque<core::VisualizationData> viz_queue_;

    OdometryCallback odom_callback_;
    VisualizationCallback viz_callback_;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
