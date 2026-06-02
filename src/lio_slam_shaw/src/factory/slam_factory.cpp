#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/lio_initializer_factory.hpp"
#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"
#include "lio_slam_shaw/factory/odometry_estimator_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

namespace lio_slam_shaw::factory {

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node* node,
                                                   const Extrinsics& extrinsics) {
    auto lio_initializer =
        LioInitializerFactory::create(node, extrinsics.T_base_lidar, extrinsics.T_base_imu);

    auto scan_preprocessor = ScanPreprocessorFactory::create(node, extrinsics.T_base_lidar);

    auto feature_extractor = FeatureExtractorFactory::create(node);

    auto local_map = LocalMapBuilderFactory::create(node, extrinsics.T_base_lidar);

    auto global_map = GlobalMapBuilderFactory::create(node, extrinsics.T_base_lidar);

    auto odometry_estimator = OdometryEstimatorFactory::create(node, extrinsics.T_base_lidar,
                                                               extrinsics.T_base_imu, local_map);

    core::FrontEndParams frontend_params;
    frontend_params.max_pending_lidar_queue =
        static_cast<size_t>(node->declare_parameter<int>("frontend.max_pending_lidar_queue", 0));

    auto frontend =
        std::make_shared<core::FrontEnd>(scan_preprocessor, feature_extractor, odometry_estimator,
                                         local_map, lio_initializer, frontend_params);

    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = LoopClosureDetectorFactory::create(node, extrinsics.T_base_lidar);

    auto backend =
        std::make_shared<core::BackEnd>(global_map, map_optimizer, loop_closure_detector);

    return std::make_shared<core::SlamProcessor>(frontend, backend);
}

}  // namespace lio_slam_shaw::factory
