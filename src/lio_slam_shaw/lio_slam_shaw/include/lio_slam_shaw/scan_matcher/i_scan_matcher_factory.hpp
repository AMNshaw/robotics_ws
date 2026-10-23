#ifndef LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_FACTORY_HPP_
#define LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_FACTORY_HPP_

#include <Eigen/Dense>

#include "lio_slam_shaw/core/i_scan_matcher.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::scan_matcher {

class IScanMatcherFactory {
public:
    virtual ~IScanMatcherFactory() = default;
    virtual core::IScanMatcher::SharedPtr create(
        const Eigen::Isometry3d& T_base_lidar,
        core::ILocalMapBuilder::SharedPtr local_map) const = 0;

private:
    virtual core::IScanMatcher::SharedPtr createIkdTreeScanMatcher(
        const Eigen::Isometry3d& T_base_lidar,
        std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map) const = 0;
};

}  // namespace lio_slam_shaw::scan_matcher

#endif  // LIO_SLAM_SHAW__SCAN_MATCHER__I_SCAN_MATCHER_FACTORY_HPP_
