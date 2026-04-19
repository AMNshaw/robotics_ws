#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"
#include "lio_slam_shaw/factory/odometry_estimator_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

namespace lio_slam_shaw::factory {

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node* node,
                                                   const Extrinsics& extrinsics) {
    auto sensor_data_manager = std::make_shared<core::SensorDataManager>();

    auto scan_preprocessor = ScanPreprocessorFactory::create(node);

    auto feature_extractor = FeatureExtractorFactory::create(node);

    auto map_builder = MapBuilderFactory::create(node);

    auto odometry_estimator = OdometryEstimatorFactory::create(node, extrinsics.T_base_lidar,
                                                               extrinsics.T_base_imu, map_builder);

    auto frontend = std::make_shared<core::FrontEnd>(sensor_data_manager, scan_preprocessor,
                                                     feature_extractor, odometry_estimator);

    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = LoopClosureDetectorFactory::create(node, extrinsics.T_base_lidar);

    auto backend =
        std::make_shared<core::BackEnd>(map_builder, map_optimizer, loop_closure_detector);

    return std::make_shared<core::SlamProcessor>(frontend, backend, map_builder);
}

}  // namespace lio_slam_shaw::factory
