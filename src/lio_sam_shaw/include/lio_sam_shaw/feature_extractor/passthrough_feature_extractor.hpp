#ifndef LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_
#define LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_

#include "lio_slam_shaw/core/i_feature_extractor.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::feature_extractor {

// 用於 ikd-Tree 類匹配器（如 IkdTreeScanMatcher）的 feature extractor。
// 不做任何幾何特徵提取，只將 LidarData::cloud 的 shared_ptr 直接 alias 給
// FeatureSet::raw_deskewed，達到 zero-copy。
// edge / surf 維持空點雲（預設值），不影響使用 raw_deskewed 的下游。
class PassthroughFeatureExtractor : public core::IFeatureExtractor {
public:
    PassthroughFeatureExtractor() = default;
    ~PassthroughFeatureExtractor() override = default;

    core::FeatureSet extract(const core::LidarData& deskewed_lidar) override;
};

}  // namespace lio_slam_shaw::feature_extractor

#endif  // LIO_SLAM_SHAW__FEATURE_EXTRACTOR__PASSTHROUGH_FEATURE_EXTRACTOR_HPP_
