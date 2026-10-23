#ifndef LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_FACTORY_HPP_

#include <Eigen/Geometry>

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

enum class ScanPreprocessorType { IMU_DESKEW };

class IScanPreprocessorFactory {
public:
    virtual ~IScanPreprocessorFactory() = default;
    virtual core::IScanPreprocessor::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                                      ScanPreprocessorType type) const = 0;

private:
    virtual core::IScanPreprocessor::SharedPtr createDeskew(
        const Eigen::Isometry3d& T_base_lidar) const = 0;
};

}  // namespace lio_slam_shaw::lidar_preprocessor

#endif  // LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_FACTORY_HPP_
