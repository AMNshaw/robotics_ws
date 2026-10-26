#include "lio_slam_shaw_ros2/factory/feature_extractor_factory.hpp"

#include <stdexcept>

#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw_ros2::factory {

lio_slam_shaw::core::IFeatureExtractor::SharedPtr RosFeatureExtractorFactory::create(
    lio_slam_shaw::feature_extractor::FeatureExtractorType type) const {
    switch (type) {
        case slam::feature_extractor::FeatureExtractorType::PASSTHROUGH:
            return std::make_shared<slam::feature_extractor::PassthroughFeatureExtractor>();
    }
    throw std::invalid_argument("FeatureExtractorFactory: unknown type");
}

}  // namespace lio_slam_shaw_ros2::factory
