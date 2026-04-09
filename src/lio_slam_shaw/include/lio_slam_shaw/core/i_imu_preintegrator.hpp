#ifndef LIO_SLAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_
#define LIO_SLAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_

#include <optional>
#include <vector>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class IImuPreintegrator {
public:
    using SharedPtr = std::shared_ptr<IImuPreintegrator>;
    using ConstSharedPtr = std::shared_ptr<const IImuPreintegrator>;

    virtual ~IImuPreintegrator() = default;

    virtual void integrateImusAndPredict(const std::vector<core::ImuData>& imus) = 0;

    virtual void updateBiasAndRepropagateImus(
        const core::NavState& optimized_state, const std::vector<core::ImuData>& opt_imu_segment,
        const std::vector<core::ImuData>& reprop_imu_segment) = 0;

    virtual NavState getLatestNavState() const = 0;

    // 查詢指定時間點的 NavState，以歷史紀錄插值取得
    // 若 queue 為空或時間超出範圍，回傳 nullopt
    virtual std::optional<NavState> queryNavState(const Timestamp& t) const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_IMU_PREINTEGRATOR_HPP_