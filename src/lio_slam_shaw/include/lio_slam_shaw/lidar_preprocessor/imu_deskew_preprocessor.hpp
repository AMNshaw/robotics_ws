#ifndef LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_
#define LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

struct ImuDeskewPreprocessorParams {
    // 0 或負值代表不做 downsample
    float voxel_leaf_size = 0.2f;
};

class ImuDeskewPreprocessor : public core::IScanPreprocessor {
public:
    explicit ImuDeskewPreprocessor(
        const ImuDeskewPreprocessorParams& params = ImuDeskewPreprocessorParams{});
    ~ImuDeskewPreprocessor() override = default;

    core::LidarData processCloud(const std::vector<core::NavState>& nav_state_snapshot,
                                 const core::LidarData& raw_data) override;

private:
    ImuDeskewPreprocessorParams params_;
};

}  // namespace lio_slam_shaw::lidar_preprocessor

#endif  // LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__IMU_DESKEW_PREPROCESSOR_HPP_