#include "lio_slam_shaw/factory/backend_facade.hpp"

#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"

namespace lio_slam_shaw::factory {

core::BackEnd::SharedPtr GtsamBackEndFacade::create(rclcpp::Node* node,
                                                    const Extrinsics& extrinsics) {
    auto global_map =
        GlobalMapBuilderFactory::create(node, extrinsics.T_base_lidar, GlobalMapType::KEYFRAME);
    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = LoopClosureDetectorFactory::create(node);

    return std::make_shared<core::BackEnd>(global_map, map_optimizer, loop_closure_detector);
}

}  // namespace lio_slam_shaw::factory
