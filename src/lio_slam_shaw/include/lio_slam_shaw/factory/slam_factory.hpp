#ifndef LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/slam_processor.hpp"

namespace lio_slam_shaw::factory {

// 一鍵組裝整個 SlamProcessor：
//   ImuPreintegrator → ScanPreprocessor → FeatureExtractor
//   → ScanMatcher + MapBuilder → FrontEnd
//   → MapOptimizer + LoopClosureDetector → BackEnd
//   → SlamProcessor
//
// 所有子元件的型別與參數從 ROS params 讀取
class SlamFactory {
public:
    static core::SlamProcessor::SharedPtr create(rclcpp::Node::SharedPtr node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SLAM_FACTORY_HPP_
