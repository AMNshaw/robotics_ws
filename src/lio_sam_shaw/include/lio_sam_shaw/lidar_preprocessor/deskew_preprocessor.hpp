#ifndef LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_
#define LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_

#include "lio_sam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_sam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_sam_shaw::lidar_preprocessor {

class DeskewPreprocessor : public IScanPreprocessor {
public:
    explicit DeskewPreprocessor(core::IImuPreintegrator::SharedPtr imu_preintegrator);
    ~DeskewPreprocessor() override = default;

private:
}

}  // namespace lio_sam_shaw::lidar_preprocessor

#endif  // LIO_SAM_SHAW__LIDAR_PREPROCESSOR__DESKEW_PREPROCESSOR_HPP_