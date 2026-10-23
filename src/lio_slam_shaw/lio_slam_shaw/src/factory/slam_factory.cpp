#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/factory/backend_facade.hpp"
#include "lio_slam_shaw/factory/frontend_facade.hpp"

namespace lio_slam_shaw::factory {

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node* node,
                                                   const Extrinsics& extrinsics) {
    FastLIOFacade frontend_facade;
    auto frontend = frontend_facade.create(node, extrinsics);

    GtsamBackEndFacade backend_facade;
    auto backend = backend_facade.create(node, extrinsics);

    return std::make_shared<core::SlamProcessor>(frontend, backend);
}

}  // namespace lio_slam_shaw::factory
