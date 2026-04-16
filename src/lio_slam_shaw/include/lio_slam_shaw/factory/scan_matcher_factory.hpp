#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class ScanMatcherFactory {
public:
    static core::IScanMatcher::SharedPtr create(rclcpp::Node* node,
                                                const Eigen::Isometry3d& T_base_lidar,
                                                core::IMapBuilder::SharedPtr map_builder);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
