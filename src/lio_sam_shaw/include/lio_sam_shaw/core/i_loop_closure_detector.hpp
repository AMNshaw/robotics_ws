#ifndef LIO_SAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_sam_shaw/core/i_map_builder.hpp"
#include "lio_sam_shaw/core/sensor_data_types.hpp"

namespace lio_sam_shaw::core {

// Loop closure 的約束資料，由 ILoopClosureDetector 產生，傳入 IMapOptimizer
struct LoopConstraint {
    uint64_t from_id;  // 當前幀 keyframe id
    uint64_t to_id;    // 历史幀 keyframe id

    // 相對位姿: T_to_from，即從 from 到 to 的變換
    Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;

    double fitness_score = 0.0;  // ICP fitness，越小越好
};

class ILoopClosureDetector {
public:
    using SharedPtr = std::shared_ptr<ILoopClosureDetector>;
    using ConstSharedPtr = std::shared_ptr<const ILoopClosureDetector>;

    virtual ~ILoopClosureDetector() = default;

    // 以當前 keyframe 對歷史地圖偵測 loop closure
    // 若找到，回傳 LoopConstraint；未找到回傳 nullopt
    virtual std::optional<LoopConstraint> detect(const KeyFrame::SharedPtr& current_keyframe,
                                                 const IMapBuilder& map_builder) = 0;
};

}  // namespace lio_sam_shaw::core

#endif  // LIO_SAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_
