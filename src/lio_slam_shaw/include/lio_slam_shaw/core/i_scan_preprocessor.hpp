#ifndef LIO_SLAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_
#define LIO_SLAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_

#include <memory>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class IScanPreprocessor {
public:
    using SharedPtr = std::shared_ptr<IScanPreprocessor>;
    using ConstSharedPtr = std::shared_ptr<const IScanPreprocessor>;

    IScanPreprocessor() = default;
    virtual ~IScanPreprocessor() = default;

    virtual LidarData processCloud(const std::vector<NavState>& nav_state_snapshot,
                                   const LidarData& raw_data) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_SCAN_PROCESSOR_HPP_