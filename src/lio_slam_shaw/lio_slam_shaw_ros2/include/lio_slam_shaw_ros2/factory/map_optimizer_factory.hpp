#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_optimizer/i_map_optimizer_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

namespace core = lio_slam_shaw::core;
namespace map_optimizer = lio_slam_shaw::map_optimizer;

class RosMapOptimizerFactory : public map_optimizer::IMapOptimizerFactory {
public:
    RosMapOptimizerFactory(rclcpp::Node* node) : node_(node) {}

    core::IMapOptimizer::SharedPtr create(map_optimizer::MapOptimizerType type) const override;

private:
    core::IMapOptimizer::SharedPtr createGtsamMapOptimizer() const;

    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
