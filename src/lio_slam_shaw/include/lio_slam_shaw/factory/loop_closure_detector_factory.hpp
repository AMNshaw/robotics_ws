#ifndef LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class LoopClosureDetectorFactory {
public:
    // type 由 ROS param "loop_closure_detector_type" 控制
    // 支援: "icp" (預設), "ikd_tree"
    // scan_matcher 僅 "ikd_tree" 使用，"icp" 可傳 nullptr
    static core::ILoopClosureDetector::SharedPtr create(
        rclcpp::Node::SharedPtr node, core::IScanMatcher::SharedPtr scan_matcher = nullptr);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
