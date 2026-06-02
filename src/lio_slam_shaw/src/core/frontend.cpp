#include "lio_slam_shaw/core/frontend.hpp"

#include <iostream>

#include "lio_slam_shaw/utils/tiktok.hpp"

namespace lio_slam_shaw::core {

FrontEnd::FrontEnd(IScanPreprocessor::SharedPtr scan_preprocessor,
                   IFeatureExtractor::SharedPtr feature_extractor,
                   IOdometryEstimator::SharedPtr odometry_estimator,
                   ILocalMapBuilder::SharedPtr local_map, ILioInitializer::SharedPtr initializer,
                   const FrontEndParams& params)
    : scan_preprocessor_(std::move(scan_preprocessor)),
      feature_extractor_(std::move(feature_extractor)),
      odometry_estimator_(std::move(odometry_estimator)),
      local_map_(std::move(local_map)),
      initializer_(std::move(initializer)),
      params_(params),
      initialized_(initializer_ == nullptr) {
    if (initializer_) {
        init_thread_ = std::thread(&FrontEnd::initThread, this);
    }
}

FrontEnd::~FrontEnd() {
    // Wake init thread so it can exit
    initialized_.store(true);
    init_cv_.notify_all();
    if (init_thread_.joinable()) init_thread_.join();
}

void FrontEnd::initThread() {
    constexpr int kMaxRetries = 3;
    int retry_count = 0;

    while (!initialized_.load()) {
        // Wait until enough scans are buffered (or shutdown)
        {
            std::unique_lock<std::mutex> lock(init_cv_mutex_);
            init_cv_.wait(lock, [this] {
                return initialized_.load() || (initializer_ && initializer_->hasEnoughData());
            });
        }

        if (initialized_.load()) break;

        if (initializer_->tryInitialize()) {
            const auto init_result = initializer_->getResult();
            odometry_estimator_->setInitialState(init_result);

            // Replay any IMU samples buffered after the init timestamp so the
            // iEKF has no gap between init completion and first scan.
            for (const auto& imu : initializer_->getImuBuffer()) {
                if (imu.timestamp > init_result.timestamp) {
                    odometry_estimator_->feedImu(imu);
                }
            }

            initialized_.store(true);
            std::clog << "[FrontEnd] SFM init succeeded — switching to iEKF\n";
            break;
        }

        // tryInitialize() failed (degenerate scene, poor gravity estimate, etc.)
        ++retry_count;
        std::clog << "[FrontEnd] Init attempt " << retry_count << " failed";

        if (retry_count >= kMaxRetries) {
            // Give up on SFM — fall back to IMU static gravity alignment
            std::clog << " — max retries reached, using static IMU fallback\n";
            const auto init_result = staticImuFallback();
            odometry_estimator_->setInitialState(init_result);

            // Replay post-init IMU (all buffered, since static fallback uses
            // early samples — everything after its timestamp is useful).
            for (const auto& imu : initializer_->getImuBuffer()) {
                if (imu.timestamp > init_result.timestamp) {
                    odometry_estimator_->feedImu(imu);
                }
            }

            initialized_.store(true);
            break;
        }

        // Drop stale scans and let fresh ones accumulate before next attempt
        std::clog << " — clearing scan buffer, re-accumulating\n";
        initializer_->clearScans();
    }
}

void FrontEnd::feed_lidar(const LidarData& lidar) {
    if (!initialized_.load()) {
        if (initializer_) {
            initializer_->addScan(lidar);
            init_cv_.notify_one();
        }
        return;
    }
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    pending_lidar_.push_back(lidar);
}

void FrontEnd::feed_imu(const ImuData& imu) {
    if (!initialized_.load()) {
        if (initializer_) {
            initializer_->addImu(imu);
        }
        return;
    }
    last_imu_time_ = imu.timestamp;
    odometry_estimator_->feedImu(imu);
}

NavState FrontEnd::getLatestOdomState() const { return odometry_estimator_->getLatestState(); }

LioInitResult FrontEnd::staticImuFallback() const {
    LioInitResult res;  // defaults: R=I, p=0, v=0, b=0, g={0,0,-9.80511}

    const auto& imu_buf = initializer_->getImuBuffer();
    if (imu_buf.empty()) {
        std::clog << "[FrontEnd] static fallback: no IMU data — using identity state\n";
        return res;
    }

    // Average the first min(N, 500) accel samples (~1 s @ 500 Hz)
    constexpr size_t kMaxSamples = 500;
    const size_t n = std::min(imu_buf.size(), kMaxSamples);
    Eigen::Vector3d mean_acc = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < n; ++i) mean_acc += imu_buf[i].acc;
    mean_acc /= static_cast<double>(n);

    // At rest: acc_body ≈ R_body_world * (-gravity_world). Choose R_world_body
    // so that R_world_body * mean_acc + gravity_world = 0.
    const double mag = mean_acc.norm();

    if (mag < 1e-3) {
        std::clog << "[FrontEnd] static fallback: degenerate accel mean — using identity\n";
        return res;
    }

    constexpr double kG = 9.80511;
    const Eigen::Vector3d gravity_world(0.0, 0.0, -kG);
    const Eigen::Vector3d acc_world = -gravity_world;
    const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(mean_acc, acc_world);
    res.R = q.toRotationMatrix();
    res.gravity = gravity_world;
    res.timestamp = imu_buf.back().timestamp;

    std::clog << "[FrontEnd] static fallback: |mean_acc|=" << mag << " m/s² from " << n
              << " IMU samples\n";
    return res;
}

bool FrontEnd::SensorDataSynced() {
    if (!initialized_.load()) return false;
    std::lock_guard<std::mutex> lock(lidar_mutex_);
    return !pending_lidar_.empty() && last_imu_time_ > pending_lidar_.front().time_end;
}

std::optional<LidarFrame::SharedPtr> FrontEnd::processPipeline() {
    std::lock_guard<std::mutex> lock(pipeline_mtx_);
    utils::TikTok timer("FrontEnd");

    LidarData lidar;
    {
        std::lock_guard<std::mutex> llock(lidar_mutex_);
        if (pending_lidar_.empty()) return std::nullopt;

        // If explicitly configured for live real-time, drop stale frames and keep
        // the latest. Disabled by default for dataset evaluation: FAST-LIO-style
        // scan-to-map odometry assumes consecutive scans are processed.
        if (params_.max_pending_lidar_queue > 0 &&
            pending_lidar_.size() > params_.max_pending_lidar_queue) {
            const size_t dropped = pending_lidar_.size() - 1;
            lidar = std::move(pending_lidar_.back());
            pending_lidar_.clear();
            std::clog << "[FrontEnd] Dropped " << dropped << " stale frames to stay real-time\n";
        } else {
            lidar = std::move(pending_lidar_.front());
            pending_lidar_.pop_front();
        }
    }

    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    // 1. Deskew using predicted nav states from the odometry estimator
    auto nav_snapshot = odometry_estimator_->getNavStateQueueSnapshot();
    auto processed_cloud = scan_preprocessor_->processCloud(nav_snapshot, lidar);
    auto t1 = Clock::now();

    // 2. Feature extraction
    auto features = feature_extractor_->extract(processed_cloud);
    auto t2 = Clock::now();

    // 3. Odometry estimation (iEKF update + IMU repropagate — all internal)
    auto odom_result = odometry_estimator_->estimateWithFeatures(features, lidar.timestamp);
    auto t3 = Clock::now();

    size_t q_size = 0;
    {
        std::lock_guard<std::mutex> llock(lidar_mutex_);
        q_size = pending_lidar_.size();
    }

    if (!odom_result.matched_in_map.is_converged) {
        std::clog << "[FrontEnd] DIVERGED | cloud=" << lidar.cloud->size()
                  << " processed=" << (features.raw_deskewed ? features.raw_deskewed->size() : 0)
                  << " q=" << q_size << '\n';
        return std::nullopt;
    }

    // 4. Build LidarFrame
    auto frame = LidarFrame::make_frame(frame_id_counter_++, lidar.timestamp, lidar.cloud,
                                        processed_cloud.cloud, features, odom_result.matched_in_map,
                                        odom_result.corrected_state);

    // 5. Update local map (addScan includes internal box trim)
    local_map_->addScan(frame);
    auto t4 = Clock::now();

    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    double total_ms = timer.tok();
    std::clog << " deskew=" << ms(t0, t1) << " feat=" << ms(t1, t2) << " odom=" << ms(t2, t3)
              << " map=" << ms(t3, t4)
              << " | pts=" << (features.raw_deskewed ? features.raw_deskewed->size() : 0)
              << " q=" << q_size << '\n';
    (void)total_ms;
    return frame;
}

}  // namespace lio_slam_shaw::core