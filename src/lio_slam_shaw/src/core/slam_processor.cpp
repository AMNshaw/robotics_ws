#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw::core {

SlamProcessor::SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend,
                             IMapBuilder::SharedPtr map_builder)
    : front_end_(std::move(frontend)),
      back_end_(std::move(backend)),
      map_builder_(std::move(map_builder)) {}

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

        std::optional<KeyFrame::SharedPtr> keyframe_to_push;
        {
            std::shared_lock<std::shared_mutex> map_lock(map_mutex_);
            std::optional<LidarFrame::SharedPtr> lidar_frame_opt = front_end_->processPipeline();
            if (!lidar_frame_opt.has_value()) continue;
            keyframe_to_push = map_builder_->addFrame(lidar_frame_opt.value());
        }
        if (keyframe_to_push.has_value()) {
            std::lock_guard<std::mutex> backend_lock(backend_mutex_);
            keyframe_queue_.push(keyframe_to_push.value());
            backend_cv_.notify_one();
        }
    }
}

void SlamProcessor::backendThread() {
    while (run_.load()) {
        KeyFrame::SharedPtr keyframe;
        {
            std::unique_lock<std::mutex> lock(backend_mutex_);
            backend_cv_.wait(lock, [this] { return !run_.load() || !keyframe_queue_.empty(); });
            if (!run_.load()) break;

            keyframe = keyframe_queue_.front();
            keyframe_queue_.pop();
        }

        back_end_->processKeyframe(keyframe);
        std::optional<Eigen::Isometry3d> global_correction = back_end_->updateGlobalCorrection();
        if (global_correction.has_value()) {
            std::unique_lock<std::shared_mutex> map_lock(map_mutex_);
            back_end_->updateMap();
            front_end_->applyOdomToMapCorrection(global_correction.value());
        }
    }
}

}  // namespace lio_slam_shaw::core
