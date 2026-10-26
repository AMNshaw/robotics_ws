#ifndef LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_CONFIG_PROVIDER_HPP_
#define LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_CONFIG_PROVIDER_HPP_

#include <variant>

#include "lio_slam_shaw/core/i_config_provider.hpp"
#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw::feature_extractor {

using FeatureExtractorConfig = std::variant<PassthroughFeatureExtractor::Config>;

class IPassthroughFeatureExtractorConfigProvider
    : core::IConfigProvider<PassthroughFeatureExtractor::Config> {
public:
    virtual ~IPassthroughFeatureExtractorConfigProvider() = default;
    virtual PassthroughFeatureExtractor::Config load() const = 0;
};

}  // namespace lio_slam_shaw::feature_extractor

#endif  // LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_CONFIG_PROVIDER_HPP_
