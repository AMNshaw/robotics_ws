#ifndef LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class LoopClosureDetectorFactory {
public:
    static core::ILoopClosureDetector::SharedPtr create(rclcpp::Node* node,
                                                        const Eigen::Isometry3d& T_base_lidar);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__LOOP_CLOSURE_DETECTOR_FACTORY_HPP_
