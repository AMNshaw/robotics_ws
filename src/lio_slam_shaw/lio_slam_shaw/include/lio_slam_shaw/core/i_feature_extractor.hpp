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

    virtual FeatureSet extract(const LidarData& deskewed_lidar) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_
