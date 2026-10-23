#include "lio_slam_shaw/factory/frontend_facade.hpp"

#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"
#include "lio_slam_shaw/factory/lio_initializer_factory.hpp"
#include "lio_slam_shaw/factory/map_builder_factory.hpp"
#include "lio_slam_shaw/factory/odometry_estimator_factory.hpp"
#include "lio_slam_shaw/factory/scan_preprocessor_factory.hpp"

namespace lio_slam_shaw::factory {

core::FrontEnd::SharedPtr FastLIOFacade::create(rclcpp::Node* node, const Extrinsics& extrinsics) {
    auto scan_preprocessor = ScanPreprocessorFactory::create(node, extrinsics.T_base_lidar,
                                                             ScanPreprocessorType::IMU_DESKEW);

    auto feature_extractor =
        FeatureExtractorFactory::create(node, FeatureExtractorType::PASSTHROUGH);

    local_map_ =
        LocalMapBuilderFactory::create(node, extrinsics.T_base_lidar, LocalMapType::IKD_TREE);

    auto odometry_estimator =
        OdometryEstimatorFactory::create(node, extrinsics.T_base_lidar, extrinsics.T_base_imu,
                                         local_map_, OdometryEstimatorType::FAST_LIO);

    auto lio_initializer =
        LioInitializerFactory::create(node, extrinsics.T_base_lidar, extrinsics.T_base_imu);

    core::FrontEndParams frontend_params;
    frontend_params.max_pending_lidar_queue =
        static_cast<size_t>(node->declare_parameter<int>("frontend.max_pending_lidar_queue", 0));

    return std::make_shared<core::FrontEnd>(scan_preprocessor, feature_extractor,
                                            odometry_estimator, local_map_, lio_initializer,
                                            frontend_params);
}

}  // namespace lio_slam_shaw::factory
