#ifndef LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/map_optimizer/gtsam_map_optimizer.hpp"

namespace lio_slam_shaw::map_optimizer {

using MapOptimizerConfig = std::variant<GtsamMapOptimizer::Config>;

class IGtsamMapOptimizerConfigProvider : core::IConfigProvider<GtsamMapOptimizer::Config> {
public:
    virtual ~IGtsamMapOptimizerConfigProvider() = default;
    virtual GtsamMapOptimizer::Config load() const = 0;
};

}  // namespace lio_slam_shaw::map_optimizer

#endif  // LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_CONFIG_PROVIDER_HPP_
