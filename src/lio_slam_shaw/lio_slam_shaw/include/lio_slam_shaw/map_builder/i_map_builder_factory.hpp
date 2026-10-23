#ifndef LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_FACTORY_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_FACTORY_HPP_

#include "lio_slam_shaw/core/i_global_map_builder.hpp"
#include "lio_slam_shaw/core/i_local_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

namespace lio_slam_shaw::map_builder {

enum class LocalMapType { IKD_TREE };
enum class GlobalMapType { KEYFRAME };

class ILocalMapBuilderFactory {
public:
    virtual ~ILocalMapBuilderFactory() = default;
    virtual core::ILocalMapBuilder::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                                     LocalMapType type) const = 0;
};

class IGlobalMapBuilderFactory {
public:
    virtual ~IGlobalMapBuilderFactory() = default;
    virtual core::IGlobalMapBuilder::SharedPtr create(const Eigen::Isometry3d& T_base_lidar,
                                                      GlobalMapType type) const = 0;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_FACTORY_HPP_
