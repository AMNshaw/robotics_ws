#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw::core {

SlamProcessor::SlamProcessor(FrontEnd::SharedPtr frontend, BackEnd::SharedPtr backend)
    : front_end_(std::move(frontend)), back_end_(std::move(backend)) {}

SlamProcessor::~SlamProcessor() {
    run_.store(false);
    sync_cv_.notify_all();
    backend_cv_.notify_all();
    local_viz_cv_.notify_all();
    global_viz_cv_.notify_all();

    if (frontend_thread_.joinable()) frontend_thread_.join();
    if (backend_thread_.joinable()) backend_thread_.join();
    if (local_viz_thread_.joinable()) local_viz_thread_.join();
    if (global_viz_thread_.joinable()) global_viz_thread_.join();
}
void SlamProcessor::start() {
    run_.store(true);
    frontend_thread_ = std::thread(&SlamProcessor::frontendThread, this);
    backend_thread_ = std::thread(&SlamProcessor::backendThread, this);
    local_viz_thread_ = std::thread(&SlamProcessor::localVizThread, this);
    global_viz_thread_ = std::thread(&SlamProcessor::globalVizThread, this);
}

void SlamProcessor::feedLidar(const LidarData& lidar) {
    front_end_->feed_lidar(lidar);
    sync_cv_.notify_one();
}

void SlamProcessor::feedImu(const ImuData& imu) {
    front_end_->feed_imu(imu);
    sync_cv_.notify_one();

    if (odom_callback_) {
        const auto state = front_end_->getLatestOdomState();
        odom_callback_(state);
    }
}

void SlamProcessor::registerOdometryCallback(const SlamProcessor::OdometryCallback& callback) {
    odom_callback_ = callback;
}

void SlamProcessor::registerLocalVizCallback(const SlamProcessor::LocalVizCallback& callback) {
    local_viz_callback_ = callback;
}

void SlamProcessor::registerGlobalVizCallback(const SlamProcessor::GlobalVizCallback& callback) {
    global_viz_callback_ = callback;
}

void SlamProcessor::frontendThread() {
    while (run_.load()) {
        std::unique_lock<std::mutex> lock(sync_mutex_);
        sync_cv_.wait(lock, [this] { return !run_.load() || front_end_->SensorDataSynced(); });

        if (!run_.load()) break;

        std::optional<Keyframe::SharedPtr> keyframe_to_push;
        {
            std::shared_lock<std::shared_mutex> map_lock(map_mutex_);
            std::optional<LidarFrame::SharedPtr> lidar_frame_opt = front_end_->processPipeline();

            if (!lidar_frame_opt.has_value()) continue;
            auto lidar_frame = lidar_frame_opt.value();

            // Try to add keyframe via backend
            keyframe_to_push = back_end_->tryAddKeyframe(lidar_frame);

            {
                std::lock_guard<std::mutex> viz_lock(local_viz_mutex_);
                local_viz_queue_.emplace_back(LocalVizData{lidar_frame->timestamp,
                                                           lidar_frame->state_odom.pose,
                                                           lidar_frame->deskewed_cloud});

                if (local_viz_queue_.size() > 5) {
                    local_viz_queue_.pop_front();
                }
            }
            local_viz_cv_.notify_one();
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
        Keyframe::SharedPtr keyframe;
        {
            std::unique_lock<std::mutex> lock(backend_mutex_);
            backend_cv_.wait(lock, [this] { return !run_.load() || !keyframe_queue_.empty(); });
            if (!run_.load()) break;

            keyframe = keyframe_queue_.front();
            keyframe_queue_.pop();
        }

        back_end_->processKeyframe(keyframe);
        back_end_->updateGlobalCorrection();

        // Push global visualization data
        ++keyframe_count_since_last_map_;
        PointCloudIRTPtr map_cloud = nullptr;
        if (keyframe_count_since_last_map_ >= global_map_publish_interval_) {
            map_cloud = back_end_->getGlobalMap();
            keyframe_count_since_last_map_ = 0;
        }
        {
            std::lock_guard<std::mutex> lock(global_viz_mutex_);
            global_viz_queue_.emplace_back(GlobalVizData{back_end_->getGlobalCorrection(),
                                                         back_end_->getAllKeyframePoses(),
                                                         back_end_->getLoopEdges(), map_cloud});

            if (global_viz_queue_.size() > 2) {
                global_viz_queue_.pop_front();
            }
        }
        global_viz_cv_.notify_one();
    }
}

void SlamProcessor::localVizThread() {
    while (run_.load()) {
        std::unique_lock<std::mutex> lock(local_viz_mutex_);
        local_viz_cv_.wait(lock, [this] { return !local_viz_queue_.empty() || !run_.load(); });

        if (!run_.load() && local_viz_queue_.empty()) break;

        auto data = local_viz_queue_.front();
        local_viz_queue_.pop_front();
        lock.unlock();

        if (local_viz_callback_) {
            local_viz_callback_(data);
        }
    }
}

void SlamProcessor::globalVizThread() {
    while (run_.load()) {
        std::unique_lock<std::mutex> lock(global_viz_mutex_);
        global_viz_cv_.wait(lock, [this] { return !global_viz_queue_.empty() || !run_.load(); });

        if (!run_.load() && global_viz_queue_.empty()) break;

        auto data = global_viz_queue_.front();
        global_viz_queue_.pop_front();
        lock.unlock();

        if (global_viz_callback_) {
            global_viz_callback_(data);
        }
    }
}

}  // namespace lio_slam_shaw::core
