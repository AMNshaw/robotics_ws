#ifndef LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_FACTORY_HPP_

#include "lio_slam_shaw/core/i_map_optimizer.hpp"

namespace lio_slam_shaw::map_optimizer {

class IMapOptimizerFactory {
public:
    virtual ~IMapOptimizerFactory() = default;
    virtual core::IMapOptimizer::SharedPtr create() const = 0;
};

}  // namespace lio_slam_shaw::map_optimizer

#endif  // LIO_SLAM_SHAW__MAP_OPTIMIZER__I_MAP_OPTIMIZER_FACTORY_HPP_
