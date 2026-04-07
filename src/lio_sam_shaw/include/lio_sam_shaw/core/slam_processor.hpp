#ifndef LIO_SAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
#define LIO_SAM_SHAW__CORE__SLAM_PROCESSOR_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

#include "lio_sam_shaw/core/backend.hpp"
#include "lio_sam_shaw/core/frontend.hpp"
#include "lio_sam_shaw/core/sensor_data_manager.hpp"
#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class SlamProcessor {
   public:
    SlamProcessor(SensorDataManager::SharedPtr data_manager, FrontEnd::SharedPtr frontend,
                  BackEnd::SharedPtr backend) {}
    ~SlamProcessor() = default;
    SlamProcessor(const SlamProcessor&) = delete;
    SlamProcessor& operator=(const SlamProcessor&) = delete;

    void processData();

    void feedLidar(const LidarData& lidar);
    std::optional<NavState> feedImu();

   private:
    void frontendThread();
    void backendThread();

    FrontEnd::SharedPtr front_end_;
    BackEnd::SharedPtr back_end_;

    std::thread frontend_thread_;
    std::thread backend_thread_;

    std::mutex sync_mutex_;
    std::condition_variable sync_cv_;

    std::mutex backend_mutex_;
    std::condition_variable backend_cv_;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__SLAM_PROCESSOR_HPP_
