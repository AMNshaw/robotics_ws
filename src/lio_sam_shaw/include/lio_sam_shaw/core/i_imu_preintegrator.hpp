#ifndef LIO_SAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_
#define LIO_SAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_

#include <vector>

#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

class IImuPreintegrator {
public:
    using SharedPtr = std::shared_ptr<IImuPreintegrator>;
    using ConstSharedPtr = std::shared_ptr<const IImuPreintegrator>;

    virtual ~IImuPreintegrator() = default;

    virtual NavState extrapolateState(const NavState& start_state,
                                      const core::ImuData& imu) const = 0;

    virtual void integrateImusAndPredict(const std::vector<core::ImuData>& imus) = 0;

    virtual void updateBiasAndRepropagateImus(
        const core::NavState& optimized_state, const std::vector<core::ImuData>& opt_imu_segment,
        const std::vector<core::ImuData>& reprop_imu_segment) = 0;

    virtual NavState getLatestNavState() const = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_