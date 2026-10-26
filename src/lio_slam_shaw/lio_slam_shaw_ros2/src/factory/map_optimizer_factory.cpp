#include "lio_slam_shaw_ros2/factory/map_optimizer_factory.hpp"

#include "lio_slam_shaw/map_optimizer/gtsam_map_optimizer.hpp"

namespace lio_slam_shaw_ros2::factory {

core::IMapOptimizer::SharedPtr RosMapOptimizerFactory::create(
    map_optimizer::MapOptimizerType type) const {
    switch (type) {
        case map_optimizer::MapOptimizerType::GTSAM:
            return createGtsamMapOptimizer();
    }
}

core::IMapOptimizer::SharedPtr RosMapOptimizerFactory::createGtsamMapOptimizer() const {
    lio_slam_shaw::Config params;

    params.odom_noise = node_->declare_parameter<std::vector<double>>(
        "backend.map_optimizer.odom_noise", {1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4});

    params.odom_noise_degenerate = node_->declare_parameter<std::vector<double>>(
        "backend.map_optimizer.odom_noise_degenerate", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    params.isam2_relinearize_threshold =
        node_->declare_parameter<double>("backend.map_optimizer.isam2_relinearize_threshold", 0.1);

    params.isam2_relinearize_skip =
        node_->declare_parameter<int>("backend.map_optimizer.isam2_relinearize_skip", 1);

    return std::make_shared<lio_slam_shaw::GtsamMapOptimizer>(params);
}

}  // namespace lio_slam_shaw_ros2::factory
