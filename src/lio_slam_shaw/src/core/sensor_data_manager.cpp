#include "lio_slam_shaw/core/sensor_data_manager.hpp"

namespace lio_slam_shaw::core {

void SensorDataManager::addImuData(const ImuData& imu) {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    if (imu.timestamp <= last_imu_time_) {
        return;
    }
    last_imu_time_ = imu.timestamp;
    imu_queue_.push_back(imu);
}

void SensorDataManager::addLidarData(const LidarData& lidar) {
    std::lock_guard<std::mutex> lock(lidar_mutex_);

    if (lidar.time_end <= last_lidar_time_) {
        return;
    }
    last_lidar_time_ = lidar.time_end;
    lidar_queue_.push_back(lidar);
}

bool SensorDataManager::getBatchImuData(const Timestamp& start_time, const Timestamp& end_time,
                                        std::vector<ImuData>& out_imu_batch) {
    std::lock_guard<std::mutex> lock(imu_mutex_);

    out_imu_batch.clear();

    while (imu_queue_.size() > 2 && imu_queue_[1].timestamp < start_time) {
        imu_queue_.pop_front();
    }

    for (const auto& imu : imu_queue_) {
        if (imu.timestamp >= start_time && imu.timestamp <= end_time) {
            out_imu_batch.push_back(imu);
        } else if (imu.timestamp > end_time) {
            break;
        }
    }

    return !out_imu_batch.empty();
}

bool SensorDataManager::getSyncedData(LidarData& out_lidar, std::vector<ImuData>& out_imu_batch) {
    std::scoped_lock lock(lidar_mutex_, imu_mutex_);

    if (lidar_queue_.empty()) return false;

    auto current_lidar = lidar_queue_.front();
    Timestamp lidar_start = current_lidar.time_start;
    Timestamp lidar_end = current_lidar.time_end;

    if (imu_queue_.empty() || imu_queue_.back().timestamp < lidar_end) {
        return false;
    }

    while (imu_queue_.size() > 1 && imu_queue_[1].timestamp <= lidar_start) {
        imu_queue_.pop_front();
    }

    out_imu_batch.clear();

    for (size_t i = 0; i < imu_queue_.size() - 1; ++i) {
        const auto& imu0 = imu_queue_[i];
        const auto& imu1 = imu_queue_[i + 1];

        if (imu0.timestamp <= lidar_start && imu1.timestamp > lidar_start) {
            out_imu_batch.push_back(interpolateImu(imu0, imu1, lidar_start));
        }

        if (imu1.timestamp > lidar_start && imu1.timestamp < lidar_end) {
            out_imu_batch.push_back(imu1);
        }

        if (imu0.timestamp < lidar_end && imu1.timestamp >= lidar_end) {
            out_imu_batch.push_back(interpolateImu(imu0, imu1, lidar_end));
            break;
        }
    }

    out_lidar = current_lidar;
    lidar_queue_.pop_front();

    return true;
}

bool SensorDataManager::hasSyncedData() const {
    std::scoped_lock lock(lidar_mutex_, imu_mutex_);

    return !lidar_queue_.empty() && !imu_queue_.empty() &&
           imu_queue_.back().timestamp >= lidar_queue_.front().time_end;
}

ImuData SensorDataManager::interpolateImu(const ImuData& imu1, const ImuData& imu2,
                                          const Timestamp& target_time) {
    double dt = getDeltaSec(imu1.timestamp, imu2.timestamp);

    if (dt < 1e-6) {
        return imu1;
    }

    double ratio = getDeltaSec(imu1.timestamp, target_time) / dt;

    Eigen::Vector3d interp_acc = imu1.acc + ratio * (imu2.acc - imu1.acc);
    Eigen::Vector3d interp_gyr = imu1.gyr + ratio * (imu2.gyr - imu1.gyr);

    return ImuData{target_time, interp_acc, interp_gyr};
}

}  // namespace lio_slam_shaw::core