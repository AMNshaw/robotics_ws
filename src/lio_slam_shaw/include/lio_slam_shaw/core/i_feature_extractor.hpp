#ifndef LIO_SLAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_
#define LIO_SLAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_

#include <memory>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class IFeatureExtractor {
public:
    using SharedPtr = std::shared_ptr<IFeatureExtractor>;
    using ConstSharedPtr = std::shared_ptr<const IFeatureExtractor>;

    virtual ~IFeatureExtractor() = default;

    // 從 deskewed LidarData 提取特徵，回傳 FeatureSet
    // 實作可選擇：
    //   - Passthrough：raw_deskewed 直接 alias LidarData::cloud（zero-copy，供 ikd-Tree 使用）
    //   - LOAM-style：從點雲提取 edge / surf 幾何特徵
    virtual FeatureSet extract(const LidarData& deskewed_lidar) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_
