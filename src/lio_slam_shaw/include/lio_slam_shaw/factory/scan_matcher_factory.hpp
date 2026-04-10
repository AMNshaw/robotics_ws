#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class ScanMatcherFactory {
public:
    // type 由 ROS param "scan_matcher_type" 控制
    // 支援: "ikd_tree" (預設)
    static core::IScanMatcher::SharedPtr create(rclcpp::Node::SharedPtr node,
                                                core::IMapBuilder::SharedPtr map_builder);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
