#ifndef LIO_SLAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SLAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_slam_shaw/core/i_global_map_builder.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct LoopConstraint {
    uint64_t from_id;
    uint64_t to_id;

    Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double, 6, 6>::Identity() * 1e-2;

    double fitness_score = 0.0;
};

class ILoopClosureDetector {
public:
    using SharedPtr = std::shared_ptr<ILoopClosureDetector>;
    using ConstSharedPtr = std::shared_ptr<const ILoopClosureDetector>;

    virtual ~ILoopClosureDetector() = default;

    virtual std::optional<LoopConstraint> detect(const Keyframe::SharedPtr& current_keyframe,
                                                 core::IGlobalMapBuilder::SharedPtr global_map) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_LOOP_CLOSURE_DETECTOR_HPP_
