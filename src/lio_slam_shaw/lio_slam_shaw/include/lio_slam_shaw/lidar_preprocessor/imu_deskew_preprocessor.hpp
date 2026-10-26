#ifndef LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_
#define LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_

#include <Eigen/Geometry>

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

class ImuDeskewPreprocessor : public core::IScanPreprocessor {
public:
    struct Config {
        float voxel_leaf_size = 0.5f;
        Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
    };

    explicit ImuDeskewPreprocessor(const Config& params = Config{});
    ~ImuDeskewPreprocessor() override = default;

    core::LidarData processCloud(const std::vector<core::NavState>& nav_state_snapshot,
                                 const core::LidarData& raw_data) override;

private:
    Config params_;
};

}  // namespace lio_slam_shaw::lidar_preprocessor

#endif  // LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_