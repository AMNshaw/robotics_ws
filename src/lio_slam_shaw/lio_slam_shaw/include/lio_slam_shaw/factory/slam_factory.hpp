#ifndef LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw::factory {

struct Extrinsics {
    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
};

class SlamFactory {
public:
    static core::SlamProcessor::SharedPtr create(rclcpp::Node* node, const Extrinsics& extrinsics);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_
