#ifndef LIO_SAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_
#define LIO_SAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_

#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class SensorDataManager {
   public:
    SensorDataManager() = default;

    void addImuData(const ImuData& imu);
    void addLidarData(const LidarData& cloud);

    bool getSyncedData(LidarData& out_cloud, std::vector<ImuData>& out_imu_batch);
    bool hasSyncedData() const;

   private:
    ImuData interpolateImu(const ImuData& imu1, const ImuData& imu2, const Timestamp& target_time);

    std::deque<ImuData> imu_queue_;
    std::deque<LidarData> lidar_queue_;  // LidarData 封裝了點雲與起訖時間

    // 執行緒鎖
    mutable std::mutex imu_mutex_;
    mutable std::mutex lidar_mutex_;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__SENSOR_DATA_MANAGER_HPP_