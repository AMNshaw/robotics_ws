#ifndef LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::factory {

class ScanMatcherFactory {
public:
    static core::IScanMatcher::SharedPtr create(
        rclcpp::Node* node, const Eigen::Isometry3d& T_base_lidar,
        std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__SCAN_MATCHER_FACTORY_HPP_
