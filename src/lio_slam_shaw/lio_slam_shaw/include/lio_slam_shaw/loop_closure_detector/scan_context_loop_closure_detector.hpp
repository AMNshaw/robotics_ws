#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/loop_closure_detector/scan_context_manager.hpp"

namespace lio_slam_shaw {

struct ScanContextLoopClosureDetectorParams {
    double min_time_gap_sec = 30.0;
    int local_map_keyframe_num = 15;
    float icp_downsample_leaf = 0.4f;
    double icp_max_corr_dist = 1.5;
    int icp_max_iterations = 100;
    double fitness_score_threshold = 0.3;
    double noise_sigma_rot = 0.1;
    double noise_sigma_trans = 0.1;
};

class ScanContextLoopClosureDetector : public core::ILoopClosureDetector {
public:
    ScanContextLoopClosureDetector(const ScanContextParams& sc_params,
                                   const ScanContextLoopClosureDetectorParams& params);
    ~ScanContextLoopClosureDetector() override = default;

    std::optional<core::LoopConstraint> detect(
        const core::Keyframe::SharedPtr& current_keyframe,
        core::IGlobalMapBuilder::SharedPtr global_map) override;

private:
    /// SC descriptor matching against history keyframes.
    std::optional<std::pair<core::Keyframe::SharedPtr, float>> findLoopCandidate(
        const core::Keyframe::SharedPtr& current_keyframe,
        const std::vector<core::Keyframe::SharedPtr>& history_keyframes);

    /// ICP verification + relative pose computation.
    std::optional<core::LoopConstraint> refineWithICP(
        const core::Keyframe::SharedPtr& current_keyframe,
        const core::Keyframe::SharedPtr& candidate, float yaw_diff_rad,
        const core::PointCloudIRTPtr& local_map) const;

    core::PointCloudIRTPtr buildLocalMap(
        const core::Keyframe::SharedPtr& candidate,
        const std::vector<core::Keyframe::SharedPtr>& history_keyframes) const;

    std::unique_ptr<ScanContextManager> sc_manager_;
    ScanContextLoopClosureDetectorParams params_;

    // Descriptor store: keyframe id -> ScDescriptor
    std::unordered_map<uint64_t, ScDescriptor> descriptors_;
};

}  // namespace lio_slam_shaw
