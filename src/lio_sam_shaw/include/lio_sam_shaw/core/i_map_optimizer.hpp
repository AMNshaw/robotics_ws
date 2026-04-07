#ifndef LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_
#define LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_

#include <memory>

#include "lio_sam_shaw/core/i_feature_extractor.hpp"
#include "lio_sam_shaw/core/sensor_data.hpp"

namespace lio_sam_shaw::core {

class IMapOptimizer {
   public:
    virtual ~IMapOptimizer() = default;

    virtual void addFeatureClouds(Timestamp timestamp, const ExtractedFeatures& features) = 0;

    virtual void addImuData(const ImuData& imu) = 0;

    virtual OdomData getLatestState() const = 0;

    virtual pcl::PointCloud<PointXYZIRT>::Ptr getGlobalMap() const = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_