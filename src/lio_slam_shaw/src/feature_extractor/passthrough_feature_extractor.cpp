#include "lio_slam_shaw/feature_extractor/passthrough_feature_extractor.hpp"

namespace lio_slam_shaw::feature_extractor {

core::FeatureSet PassthroughFeatureExtractor::extract(const core::LidarData& deskewed_lidar) {
    core::FeatureSet features;
    // zero-copy: raw_deskewed 直接 alias deskewed_lidar.cloud 的 shared_ptr
    // 兩者指向同一塊記憶體，不做任何拷貝
    features.raw_deskewed = deskewed_lidar.cloud;
    return features;
}

}  // namespace lio_slam_shaw::feature_extractor
