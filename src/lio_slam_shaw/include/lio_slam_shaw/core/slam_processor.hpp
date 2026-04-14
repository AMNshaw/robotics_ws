#ifndef LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
#define LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <shared_mutex>
#include <thread>

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class SlamProcessor {
public:
    using SharedPtr = std::shared_ptr<SlamProcessor>;

    explicit SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend,
                           IMapBuilder::SharedPtr map_builder);
    ~SlamProcessor();
    SlamProcessor(const SlamProcessor&) = delete;
    SlamProcessor& operator=(const SlamProcessor&) = delete;

    void start();

    void feedLidar(const LidarData& lidar);
    void feedImu(const ImuData& imu);

private:
    void frontendThread();
    void backendThread();

    FrontEnd::SharedPtr front_end_;
    BackEnd::SharedPtr back_end_;
    IMapBuilder::SharedPtr map_builder_;

    std::atomic<bool> run_{false};
    std::thread frontend_thread_;
    std::thread backend_thread_;

    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;

    std::mutex backend_mutex_;
    std::condition_variable backend_cv_;

    std::queue<Keyframe::SharedPtr> keyframe_queue_;
    std::shared_mutex map_mutex_;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
