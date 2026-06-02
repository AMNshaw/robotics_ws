#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/factory/frontend_facade.hpp"
#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"

namespace lio_slam_shaw::factory {

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node* node,
                                                   const Extrinsics& extrinsics) {
    // --- FrontEnd (algorithm stack fixed by Facade) ---
    FastLIOFacade frontend_facade;
    auto frontend = frontend_facade.create(node, extrinsics);

    // --- BackEnd (type still configurable via YAML) ---
    auto global_map =
        GlobalMapBuilderFactory::create(node, extrinsics.T_base_lidar, GlobalMapType::IKD_TREE);
    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = LoopClosureDetectorFactory::create(node, extrinsics.T_base_lidar);

    auto backend =
        std::make_shared<core::BackEnd>(global_map, map_optimizer, loop_closure_detector);

    return std::make_shared<core::SlamProcessor>(frontend, backend);
}

}  // namespace lio_slam_shaw::factory
