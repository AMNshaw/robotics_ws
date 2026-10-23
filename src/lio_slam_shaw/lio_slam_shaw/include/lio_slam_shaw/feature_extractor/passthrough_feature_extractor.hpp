#ifndef LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_
#define LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_

#include "lio_slam_shaw/core/i_feature_extractor.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::feature_extractor {

class PassthroughFeatureExtractor : public core::IFeatureExtractor {
public:
    PassthroughFeatureExtractor() = default;
    ~PassthroughFeatureExtractor() override = default;

    core::FeatureSet extract(const core::LidarData& deskewed_lidar) override;
};

}  // namespace lio_slam_shaw::feature_extractor

#endif  // LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_
