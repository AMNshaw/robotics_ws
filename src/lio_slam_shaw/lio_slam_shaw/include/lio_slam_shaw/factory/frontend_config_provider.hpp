#ifndef LIO_SLAM_SHAW__FACTORY__FRONTEND_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__FACTORY__FRONTEND_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"
#include "lio_slam_shaw/lidar_preprocessor/imu_deskew_preprocessor.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/odometry_estimator/fast_lio_odometry.hpp"

namespace lio_slam_shaw::factory {

struct FastLioFrontEndConfig {
    size_t max_pending_lidar_queue = 0;
    lidar_preprocessor::ImuDeskewPreprocessor::Config preprocessor_config;
    feature_extractor::PassthroughFeatureExtractor::Config extractor_config;
    map_builder::IkdTreeLocalMapBuilder::Config local_map_builder_config;
    odometry_estimator::FastLioOdometry::Config odometry_estimator_config;
};

using FrontEndConfig = std::variant<FastLioFrontEndConfig>;

}  // namespace lio_slam_shaw::factory

#endif  // LIO_SLAM_SHAW__FACTORY__FRONTEND_CONFIG_PROVIDER_HPP_