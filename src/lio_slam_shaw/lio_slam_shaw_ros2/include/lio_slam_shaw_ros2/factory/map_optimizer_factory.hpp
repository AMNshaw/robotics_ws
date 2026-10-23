#ifndef LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
#define LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_

#include <rclcpp/rclcpp.hpp>

#include "lio_slam_shaw/map_optimizer/i_map_optimizer_factory.hpp"

namespace lio_slam_shaw_ros2::factory {

class RosMapOptimizerFactory : public lio_slam_shaw::map_optimizer::IMapOptimizerFactory {
public:
    RosMapOptimizerFactory(rclcpp::Node* node);

    lio_slam_shaw::core::IMapOptimizer::SharedPtr create() const override;

private:
    rclcpp::Node* node_;
};

}  // namespace lio_slam_shaw_ros2::factory

#endif  // LIO_SLAM_SHAW_ROS2__FACTORY__MAP_OPTIMIZER_FACTORY_HPP_
