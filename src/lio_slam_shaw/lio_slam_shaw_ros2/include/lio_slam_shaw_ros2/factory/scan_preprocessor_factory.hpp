#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"
#include "lio_slam_shaw/lidar_preprocessor/i_scan_preprocessor_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosScanPreprocessorFactory
    : public lio_slam_shaw::lidar_preprocessor::IScanPreprocessorFactory {
public:
    RosScanPreprocessorFactory(rclcpp::Node* node);

    lio_slam_shaw::core::IScanPreprocessor::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar,
        lio_slam_shaw::lidar_preprocessor::ScanPreprocessorType type) const override;

private:
    lio_slam_shaw::core::IScanPreprocessor::SharedPtr createDeskew(
        const Eigen::Isometry3d& T_base_lidar) const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
