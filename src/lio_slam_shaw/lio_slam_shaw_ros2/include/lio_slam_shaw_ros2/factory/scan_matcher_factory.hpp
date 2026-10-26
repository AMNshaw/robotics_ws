#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_MATCHER_FACTORY_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/scan_matcher/i_scan_matcher_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace map_builder = lio_slam_shaw::map_builder;
namespace scan_matcher = lio_slam_shaw::scan_matcher;

class RosScanMatcherFactory : public lio_slam_shaw::scan_matcher::IScanMatcherFactory {
public:
    RosScanMatcherFactory(rclcpp::Node* node) : node_(node) {}

    lio_slam_shaw::core::IScanMatcher::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar,
        lio_slam_shaw::core::ILocalMapBuilder::SharedPtr local_map) const override;

private:
    lio_slam_shaw::core::IScanMatcher::SharedPtr createIkdTreeScanMatcher(
        const Eigen::Isometry3d& T_base_lidar,
        std::shared_ptr<lio_slam_shaw::map_builder::IkdTreeLocalMapBuilder> local_map)
        const override;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__SCAN_MATCHER_FACTORY_HPP_
