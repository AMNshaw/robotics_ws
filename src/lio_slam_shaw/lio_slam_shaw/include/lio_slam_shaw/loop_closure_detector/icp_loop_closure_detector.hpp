#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"

namespace lio_slam_shaw {

struct IcpLoopClosureDetectorParams {
    double search_radius = 15.0;
    double min_time_gap_sec = 30.0;
    int local_map_keyframe_num = 25;

    float icp_downsample_leaf = 0.4f;
    double icp_max_corr_dist = 0.3;
    int icp_max_iterations = 100;
    double fitness_score_threshold = 0.3;

    double noise_sigma_rot = 0.1;
    double noise_sigma_trans = 0.1;
};

class IcpLoopClosureDetector : public core::ILoopClosureDetector {
public:
    explicit IcpLoopClosureDetector(const IcpLoopClosureDetectorParams& params);
    ~IcpLoopClosureDetector() override = default;

    std::optional<core::LoopConstraint> detect(
        const core::Keyframe::SharedPtr& current_keyframe,
        core::IGlobalMapBuilder::SharedPtr global_map) override;

private:
    std::optional<core::Keyframe::SharedPtr> findCandidate(
        const core::Keyframe::SharedPtr& current_keyframe,
        const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const;

    core::PointCloudIRTPtr buildLocalMap(
        const core::Keyframe::SharedPtr& current_keyframe,
        const core::Keyframe::SharedPtr& candidate,
        const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const;

    IcpLoopClosureDetectorParams params_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE__ICP_LOOP_CLOSURE_DETECTOR_HPP_
