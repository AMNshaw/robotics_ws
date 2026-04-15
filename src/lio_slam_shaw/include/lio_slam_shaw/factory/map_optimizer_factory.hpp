#ifndef LIO_SLAM_SHAW__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/core/i_map_optimizer.hpp"

namespace lio_slam_shaw::factory {

class MapOptimizerFactory {
public:
    static core::IMapOptimizer::SharedPtr create(rclcpp::Node* node);
};

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
