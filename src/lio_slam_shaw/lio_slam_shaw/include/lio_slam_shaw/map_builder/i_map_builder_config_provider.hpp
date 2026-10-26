#ifndef LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/map_builder/keyframe_global_map_builder.hpp"

namespace lio_slam_shaw::map_builder {

using LocalMapConfig = std::variant<IkdTreeLocalMapBuilder::Config>;
using GlobalMapConfig = std::variant<KeyframeGlobalMapBuilder::Config>;

class IIkdTreeLocalMapBuilderConfigProvider
    : core::IConfigProvider<IkdTreeLocalMapBuilder::Config> {
public:
    virtual ~IIkdTreeLocalMapBuilderConfigProvider() = default;
    virtual IkdTreeLocalMapBuilder::Config load() const = 0;
};

class KeyframeGlobalMapBuilderConfigProvider
    : core::IConfigProvider<KeyframeGlobalMapBuilder::Config> {
public:
    virtual ~KeyframeGlobalMapBuilderConfigProvider() = default;
    virtual KeyframeGlobalMapBuilder::Config load() const = 0;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__I_MAP_BUILDER_CONFIG_PROVIDER_HPP_
