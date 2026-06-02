#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"

#include <stdexcept>

#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw::factory {

core::IFeatureExtractor::SharedPtr FeatureExtractorFactory::create(rclcpp::Node* /*node*/,
                                                                   FeatureExtractorType type) {
    switch (type) {
        case FeatureExtractorType::PASSTHROUGH:
            return std::make_shared<feature_extractor::PassthroughFeatureExtractor>();
    }
    throw std::invalid_argument("FeatureExtractorFactory: unknown type");
}

}  // namespace lio_slam_shaw::factory
