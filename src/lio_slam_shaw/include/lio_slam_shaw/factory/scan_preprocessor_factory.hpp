#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_scan_preprocessor.hpp"

namespace lio_slam_shaw::factory {

class ScanPreprocessorFactory {
public:
    static core::IScanPreprocessor::SharedPtr create(rclcpp::Node* node);

private:
    static core::IScanPreprocessor::SharedPtr createDefault();
    static core::IScanPreprocessor::SharedPtr createDeskew(rclcpp::Node* node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_PREPROCESSOR_FACTORY_HPP_
