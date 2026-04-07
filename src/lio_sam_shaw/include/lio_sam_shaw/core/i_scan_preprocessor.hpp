#ifndef LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_
#define LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_

#include <memory>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class IScanPreprocessor {
   public:
    virtual ~IScanPreprocessor() = default;

    // 接收感測器數據緩衝
    virtual void pushImuData(const ImuData& imu) = 0;
    virtual void pushOdomData(const NavState& odom) = 0;

    // 處理原始點雲，回傳去畸變後的點雲 (Deskewed Cloud)
    virtual pcl::PointCloud<PointXYZIRT>::Ptr processCloud(
        const pcl::PointCloud<PointXYZIRT>::Ptr& raw_cloud, double cloud_header_time) = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_