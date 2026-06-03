#ifndef LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
#define LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_

#include <atomic>
#include <condition_variable>
#include <deque>
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
#include "lio_slam_shaw/core/visualization_types.hpp"

namespace lio_slam_shaw::core {

class SlamProcessor {
public:
    using SharedPtr = std::shared_ptr<SlamProcessor>;
    using ConstSharedPtr = std::shared_ptr<const SlamProcessor>;

    using OdometryCallback = std::function<void(const NavState& odom_state)>;
    using LocalVizCallback = std::function<void(const LocalVizData& data)>;
    using GlobalVizCallback = std::function<void(const GlobalVizData& data)>;

    explicit SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend);
    ~SlamProcessor();
    SlamProcessor(const SlamProcessor&) = delete;
    SlamProcessor& operator=(const SlamProcessor&) = delete;

    void start();

    void feedLidar(const LidarData& lidar);
    void feedImu(const ImuData& imu);

    void registerOdometryCallback(const OdometryCallback& callback);
    void registerLocalVizCallback(const LocalVizCallback& callback);
    void registerGlobalVizCallback(const GlobalVizCallback& callback);

private:
    void frontendThread();
    void backendThread();
    void localVizThread();
    void globalVizThread();

    FrontEnd::SharedPtr front_end_;
    BackEnd::SharedPtr back_end_;

    std::atomic<bool> run_{false};
    std::thread frontend_thread_;
    std::thread backend_thread_;
    std::thread local_viz_thread_;
    std::thread global_viz_thread_;

    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;

    std::mutex backend_mutex_;
    std::condition_variable backend_cv_;

    std::queue<Keyframe::SharedPtr> keyframe_queue_;
    std::shared_mutex map_mutex_;

    std::mutex local_viz_mutex_;
    std::condition_variable local_viz_cv_;
    std::deque<LocalVizData> local_viz_queue_;

    std::mutex global_viz_mutex_;
    std::condition_variable global_viz_cv_;
    std::deque<GlobalVizData> global_viz_queue_;

    OdometryCallback odom_callback_;
    LocalVizCallback local_viz_callback_;
    GlobalVizCallback global_viz_callback_;

    size_t global_map_publish_interval_ = 5;  // assemble global map every N keyframes
    size_t keyframe_count_since_last_map_ = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
