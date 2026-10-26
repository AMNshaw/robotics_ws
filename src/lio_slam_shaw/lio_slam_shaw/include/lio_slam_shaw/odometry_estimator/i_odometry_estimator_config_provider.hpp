#ifndef LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

namespace lio_slam_shaw::odometry_estimator {

using OdometryEstimatorConfig = std::variant<FastLioOdometry>;

class IFastLioOdometryEstimatorConfigProvider : core::IConfigProvider<FastLioOdometry::Config> {
public:
    virtual ~IFastLioOdometryEstimatorConfigProvider() = default;
    virtual FastLioOdometry::Config load() const = 0;
};

}  // namespace lio_slam_shaw::odometry_estimator

#endif  // LIO_SLAM_SHAW__ODOMETRY_ESTIMATOR__I_ODOMETRY_ESTIMATOR_CONFIG_PROVIDER_HPP_
