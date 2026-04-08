#ifndef LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_
#define LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_

#include <memory>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class IScanPreprocessor {
public:
    using SharedPtr = std::shared_ptr<IScanPreprocessor>;
    using ConstSharedPtr = std::shared_ptr<const IScanPreprocessor>;

    IScanPreprocessor() = default;
    virtual ~IScanPreprocessor() = default;

    virtual LidarData processCloud(const std::vector<ImuData>& imu_data,
                                   const LidarData& raw_data) = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_