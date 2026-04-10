#ifndef LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_
#define LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

class ImuDeskewPreprocessor : public core::IScanPreprocessor {
public:
    ImuDeskewPreprocessor() = default;
    ~ImuDeskewPreprocessor() override = default;

    core::LidarData processCloud(const std::vector<core::NavState>& nav_state_snapshot,
                                 const core::LidarData& raw_data) override;
};

}  // namespace lio_slam_shaw::lidar_preprocessor

#endif  // LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_