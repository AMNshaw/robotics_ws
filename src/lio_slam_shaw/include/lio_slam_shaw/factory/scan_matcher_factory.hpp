#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/core/i_scan_matcher.hpp"

namespace lio_slam_shaw::factory {

class ScanMatcherFactory {
public:
    // Frontend 用：快速收斂，params 前綴 "scan_matcher."
    static core::IScanMatcher::SharedPtr createLocal(rclcpp::Node::SharedPtr node,
                                                     core::IMapBuilder::SharedPtr map_builder);

    // Loop closure 用：更多迭代、更嚴格收斂，params 前綴 "loop_closure.scan_matcher."
    static core::IScanMatcher::SharedPtr createGlobal(rclcpp::Node::SharedPtr node,
                                                      core::IMapBuilder::SharedPtr map_builder);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
