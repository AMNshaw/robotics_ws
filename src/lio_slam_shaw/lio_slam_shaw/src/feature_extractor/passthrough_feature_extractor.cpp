#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw::feature_extractor {

core::FeatureSet PassthroughFeatureExtractor::extract(const core::LidarData& deskewed_lidar) {
    core::FeatureSet features;
    features.raw_deskewed = deskewed_lidar.cloud;
    return features;
}

}  // namespace lio_slam_shaw::feature_extractor
