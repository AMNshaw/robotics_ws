#ifndef LIO_SLAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_
#define LIO_SLAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_

#include <memory>
#include <vector>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

class IMapOptimizer {
public:
    using SharedPtr = std::shared_ptr<IMapOptimizer>;
    using ConstSharedPtr = std::shared_ptr<const IMapOptimizer>;

    virtual ~IMapOptimizer() = default;

    virtual void addKeyframe(uint64_t keyframe_id, const ScanMatchResult& matched_result) = 0;

    virtual void addLoopConstraint(const LoopConstraint& constraint) = 0;

    virtual std::vector<std::pair<uint64_t, Eigen::Isometry3d>> optimize() = 0;

    virtual NavState getKeyframePose(uint64_t keyframe_id) const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_MAP_OPTIMIZER_HPP_