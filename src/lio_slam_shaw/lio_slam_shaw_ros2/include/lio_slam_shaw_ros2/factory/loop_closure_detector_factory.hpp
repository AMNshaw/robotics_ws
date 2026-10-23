#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/loop_closure_detector/i_loop_closure_detector_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosLoopClosureDetectorFactory
    : public lio_slam_shaw::loop_closure_detector::ILoopClosureDetectorFactory {
public:
    RosLoopClosureDetectorFactory(rclcpp::Node* node);

    virtual lio_slam_shaw::core::ILoopClosureDetector::SharedPtr create(rclcpp::Node* node) = 0;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
