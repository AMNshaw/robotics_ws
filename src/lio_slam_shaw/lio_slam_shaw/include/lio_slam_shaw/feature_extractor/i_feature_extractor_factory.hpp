#ifndef LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_FACTORY_HPP_
#define LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_FACTORY_HPP_

#include "lio_slam_shaw/core/i_feature_extractor.hpp"

namespace lio_slam_shaw::feature_extractor {

enum class FeatureExtractorType { PASSTHROUGH };

class IFeatureExtractorFactory {
public:
    virtual ~IFeatureExtractorFactory() = default;
    virtual core::IFeatureExtractor::SharedPtr create(FeatureExtractorType type) const = 0;
};

}  // namespace lio_slam_shaw::feature_extractor

#endif  // LIO_SLAM_SHAW__FEATURE_EXTRACTOR__I_FEATURE_EXTRACTOR_FACTORY_HPP_
