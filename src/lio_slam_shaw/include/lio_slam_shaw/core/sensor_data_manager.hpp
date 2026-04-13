#ifndef LIO_SLAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_
#define LIO_SLAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <vector>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class SensorDataManager {
public:
    using SharedPtr = std::shared_ptr<SensorDataManager>;
    using ConstSharedPtr = std::shared_ptr<const SensorDataManager>;

    SensorDataManager() = default;

    void addImuData(const ImuData& imu);
    void addLidarData(const LidarData& cloud);

    bool getBatchImuData(const Timestamp& start_time, const Timestamp& end_time,
                         std::vector<ImuData>& out_imu_batch);
    bool getSyncedData(LidarData& out_cloud, std::vector<ImuData>& out_imu_batch);
    bool hasSyncedData() const;

private:
    ImuData interpolateImu(const ImuData& imu1, const ImuData& imu2, const Timestamp& target_time);

    std::deque<ImuData> imu_queue_;
    std::deque<LidarData> lidar_queue_;

    Timestamp last_imu_time_{};
    Timestamp last_lidar_time_{};

    mutable std::mutex imu_mutex_;
    mutable std::mutex lidar_mutex_;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_