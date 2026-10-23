#ifndef LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"

namespace lio_slam_shaw::factory {

class ImuPreintegratorFactory {
public:
    static core::IImuPreintegrator::SharedPtr create(rclcpp::Node* node,
                                                     const Eigen::Isometry3d& T_base_imu);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_
