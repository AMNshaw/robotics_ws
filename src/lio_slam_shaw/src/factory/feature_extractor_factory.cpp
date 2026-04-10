#include "lio_slam_shaw/factory/feature_extractor_factory.hpp"

#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw::factory {

core::IFeatureExtractor::SharedPtr FeatureExtractorFactory::create(rclcpp::Node::SharedPtr node) {
    std::string type =
        node->declare_parameter<std::string>("feature_extractor_type", "passthrough");

    if (type != "passthrough") {
        RCLCPP_WARN(node->get_logger(),
                    "Unknown feature_extractor_type '%s', falling back to passthrough.",
                    type.c_str());
    }
    return std::make_shared<feature_extractor::PassthroughFeatureExtractor>();
}

}  // namespace lio_slam_shaw::factory
