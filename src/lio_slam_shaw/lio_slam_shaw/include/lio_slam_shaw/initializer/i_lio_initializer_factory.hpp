#ifndef LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_FACTORY_HPP_

#include <Eigen/Dense>

#include "lio_slam_shaw/core/i_lio_initializer.hpp"

namespace lio_slam_shaw::initializer {

class ILioInitializerFactory {
public:
    virtual ~ILioInitializerFactory() = default;
    virtual core::ILioInitializer::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                                    const Eigen::Isometry3d& T_base_imu) const = 0;
};

}  // namespace lio_slam_shaw::initializer

#endif  // LIO_SLAM_SHAW__INITIALIZER__I_LIO_INITIALIZER_FACTORY_HPP_
