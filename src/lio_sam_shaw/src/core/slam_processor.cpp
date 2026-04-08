#include "lio_sam_shaw/core/slam_processor.hpp"

namespace lio_sam_shaw::core {

SlamProcessor::SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend)
    : front_end_(std::move(frontend)), back_end_(std::move(backend)) {}

SlamProcessor::~SlamProcessor() {
    run_.store(false);
    sync_cv_.notify_all();
    backend_cv_.notify_all();

    if (frontend_thread_.joinable()) frontend_thread_.join();
    if (backend_thread_.joinable()) backend_thread_.join();
}

void SlamProcessor::start() {
    run_.store(true);
    frontend_thread_ = std::thread(&SlamProcessor::frontendThread, this);
    backend_thread_ = std::thread(&SlamProcessor::backendThread, this);
}

void SlamProcessor::feedLidar(const LidarData& lidar) {
    front_end_->feed_lidar(lidar);
    sync_cv_.notify_one();
}

void SlamProcessor::feedImu(const ImuData& imu) {
    front_end_->feed_imu(imu);
    sync_cv_.notify_one();
}

void SlamProcessor::frontendThread() {
    while (run_.load()) {
        std::unique_lock<std::mutex> lock(sync_mutex_);
        sync_cv_.wait(lock, [this] { return !run_.load() || front_end_->SensorDataSynced(); });

        if (!run_.load()) break;

        auto lidar_frame = front_end_->processPipeline();
        if (!lidar_frame.has_value()) continue;

        {
            std::lock_guard<std::mutex> backend_lock(backend_mutex_);
            frame_queue_.push(lidar_frame.value());
        }
        backend_cv_.notify_one();
    }
}

void SlamProcessor::backendThread() {
    while (run_.load()) {
        std::unique_lock<std::mutex> lock(backend_mutex_);
        backend_cv_.wait(lock, [this] { return !run_.load() || !frame_queue_.empty(); });

        if (!run_.load()) break;

        auto frame = frame_queue_.front();
        frame_queue_.pop();
        lock.unlock();

        auto correction = back_end_->processFrame(frame);
        if (correction.has_value()) {
            front_end_->updateGlobalPose(correction->first, correction->second);
        }
    }
}

}  // namespace lio_sam_shaw::core
