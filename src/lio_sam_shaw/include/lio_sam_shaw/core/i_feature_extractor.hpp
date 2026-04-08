#ifndef LIO_SAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_
#define LIO_SAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_

#include <memory>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

struct ExtractedFeatures {
    pcl::PointCloud<PointXYZIRT>::Ptr corner_cloud;
    pcl::PointCloud<PointXYZIRT>::Ptr surface_cloud;
};

class IFeatureExtractor {
public:
    using SharedPtr = std::shared_ptr<IFeatureExtractor>;
    using ConstSharedPtr = std::shared_ptr<const IFeatureExtractor>;

    virtual ~IFeatureExtractor() = default;

    virtual ExtractedFeatures extract(const pcl::PointCloud<PointXYZIRT>::Ptr& deskewed_cloud) = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_FEATURE_EXTRACTOR_HPP_