#include "lio_slam_shaw/factory/slam_factory.hpp"

#include "lio_slam_shaw/core/backend.hpp"
#include "lio_slam_shaw/core/frontend.hpp"
#include "lio_slam_shaw/core/sensor_data_manager.hpp"
#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/imu_preintegrator_factory.hpp"
#include "lio_slam_shaw/factory/loop_closure_detector_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/map_optimizer_factory.hpp"
#include "lio_slam_shaw/factory/scan_matcher_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

namespace lio_slam_shaw::factory {

core::SlamProcessor::SharedPtr SlamFactory::create(rclcpp::Node* node) {
    auto sensor_data_manager = std::make_shared<core::SensorDataManager>();
    auto imu_preintegrator = ImuPreintegratorFactory::create(node);

    auto scan_preprocessor = ScanPreprocessorFactory::create(node);

    auto feature_extractor = FeatureExtractorFactory::create(node);

    auto map_builder = MapBuilderFactory::create(node);

    auto scan_matcher = ScanMatcherFactory::create(node, map_builder);

    auto frontend = std::make_shared<core::FrontEnd>(
        sensor_data_manager, scan_preprocessor, feature_extractor, scan_matcher, imu_preintegrator);

    auto map_optimizer = MapOptimizerFactory::create(node);
    auto loop_closure_detector = LoopClosureDetectorFactory::create(node);

    auto backend =
        std::make_shared<core::BackEnd>(map_builder, map_optimizer, loop_closure_detector);

    return std::make_shared<core::SlamProcessor>(frontend, backend, map_builder);
}

}  // namespace lio_slam_shaw::factory
