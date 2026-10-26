#ifndef LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"

namespace lio_slam_shaw::lidar_preprocessor {

using ScanPreprocessorConfig = std::variant<ImuDeskewPreprocessor::Config>;

class IImuDeskewPreprocessorConfigProvider : core::IConfigProvider<ImuDeskewPreprocessor::Config> {
public:
    virtual ~IImuDeskewPreprocessorConfigProvider() = default;
    virtual ImuDeskewPreprocessor::Config load() const = 0;
};

}  // namespace lio_slam_shaw::lidar_preprocessor

#endif  // LIO_SLAM_SHAW__LIDAR_PREPROCESSOR__I_SCAN_PREPROCESSOR_CONFIG_PROVIDER_HPP_
