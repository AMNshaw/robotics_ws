#ifndef LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class LoopClosureDetectorFactory {
public:
    static core::ILoopClosureDetector::SharedPtr create(rclcpp::Node* node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
