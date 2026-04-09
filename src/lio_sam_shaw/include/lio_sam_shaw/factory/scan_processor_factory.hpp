#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_PROCESSOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_PROCESSOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"
#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_slam_shaw::factory {

class ScanProcessorFactory {
public:
    static core::IScanPreprocessor::SharedPtr createScanPreprocessor(
        rclcpp::Node::SharedPtr node, const core::IImuPreintegrator::SharedPtr& imu_preintegrator);

private:
    static core::IScanPreprocessor::SharedPtr createDefaultScanPreprocessor();
    static core::IScanPreprocessor::SharedPtr createDeskewScanPreprocessor(
        const core::IImuPreintegrator::SharedPtr& imu_preintegrator);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_PROCESSOR_FACTORY_HPP_
