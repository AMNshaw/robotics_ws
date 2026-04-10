#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"

#include "lio_slam_shaw/map_optimizer/gtsam_map_optimizer.hpp"

namespace lio_slam_shaw::factory {

core::IMapOptimizer::SharedPtr MapOptimizerFactory::create(rclcpp::Node::SharedPtr node) {
    const std::string type = node->declare_parameter<std::string>("map_optimizer_type", "gtsam");

    if (type == "gtsam") {
        GtsamMapOptimizerParams params;

        params.prior_noise = node->declare_parameter<std::vector<double>>(
            "map_optimizer.prior_noise", {1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12});

        params.odom_noise = node->declare_parameter<std::vector<double>>(
            "map_optimizer.odom_noise", {1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4});

        params.odom_noise_degenerate = node->declare_parameter<std::vector<double>>(
            "map_optimizer.odom_noise_degenerate", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

        params.isam2_relinearize_threshold =
            node->declare_parameter<double>("map_optimizer.isam2_relinearize_threshold", 0.1);

        params.isam2_relinearize_skip =
            node->declare_parameter<int>("map_optimizer.isam2_relinearize_skip", 1);

        return std::make_shared<GtsamMapOptimizer>(params);
    }

    throw std::runtime_error("MapOptimizerFactory: unknown type '" + type + "'");
}

}  // namespace lio_slam_shaw::factory
