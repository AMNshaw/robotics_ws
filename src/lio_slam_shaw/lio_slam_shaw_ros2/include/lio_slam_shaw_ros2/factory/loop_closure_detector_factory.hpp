#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/loop_closure_detector/i_loop_closure_detector_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace loop_closure_detector = lio_slam_shaw::loop_closure_detector;

class RosLoopClosureDetectorFactory : public loop_closure_detector::ILoopClosureDetectorFactory {
public:
    RosLoopClosureDetectorFactory(rclcpp::Node* node) : node_(node) {}

    core::ILoopClosureDetector::SharedPtr create(
        loop_closure_detector::LoopClosureDetectorType type) const override;

private:
    core::ILoopClosureDetector::SharedPtr createScanContext() const;
    core::ILoopClosureDetector::SharedPtr createICP() const;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
