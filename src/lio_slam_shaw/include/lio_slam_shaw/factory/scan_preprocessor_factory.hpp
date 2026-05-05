#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_slam_shaw::factory {

class ScanPreprocessorFactory {
public:
    static core::IScanPreprocessor::SharedPtr create(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar = Eigen::Isometry3d::Identity());

private:
    static core::IScanPreprocessor::SharedPtr createDefault();
    static core::IScanPreprocessor::SharedPtr createDeskew(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar = Eigen::Isometry3d::Identity());
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
