#ifndef LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"

namespace lio_slam_shaw::factory {

class ImuPreintegratorFactory {
public:
    static core::IImuPreintegrator::SharedPtr createImuPreintegrator(rclcpp::Node::SharedPtr node);

private:
    static core::IImuPreintegrator::SharedPtr createDefaultImuPreintegrator();
    static core::IImuPreintegrator::SharedPtr createImuPreintegrator(rclcpp::Node::SharedPtr node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__IMU_PREINTEGRATOR_FACTORY_HPP_
