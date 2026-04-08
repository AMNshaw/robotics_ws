#ifndef LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_
#define LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_

#include "lio_sam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_sam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::lidar_preprocessor {

class DeskewPreprocessor : public core::IScanPreprocessor {
public:
    explicit DeskewPreprocessor(core::IImuPreintegrator::SharedPtr imu_preintegrator);
    ~DeskewPreprocessor() override = default;

    core::LidarData processCloud(const std::vector<core::ImuData>& imu_data,
                                 const core::LidarData& raw_data) override;

private:
}

}  // namespace lio_sam_shaw::lidar_preprocessor

#endif  // LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_