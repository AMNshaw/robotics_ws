#ifndef LIO_SLAM_SHAW__FACTORY__LIO_INITIALIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__LIO_INITIALIZER_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_lio_initializer.hpp"

namespace lio_slam_shaw::factory {

class LioInitializerFactory {
public:
    static core::ILioInitializer::SharedPtr create(rclcpp::Node* node,
                                                   const Eigen::Isometry3d& T_base_lidar,
                                                   const Eigen::Isometry3d& T_base_imu);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__LIO_INITIALIZER_FACTORY_HPP_
