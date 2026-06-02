#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw {

struct IkdTreeLoopClosureDetectorParams {
    double search_radius = 15.0;
    double min_time_gap_sec = 30.0;
    int local_map_keyframe_num = 25;

    double fitness_score_threshold = 0.3;
};

class IkdTreeLoopClosureDetector : public core::ILoopClosureDetector {
public:
    IkdTreeLoopClosureDetector(const scan_matcher::IkdTreeScanMatcherParams& scan_matcher_params,
                               const IkdTreeLoopClosureDetectorParams& loop_closure_params);
    ~IkdTreeLoopClosureDetector() override = default;

    std::optional<core::LoopConstraint> detect(
        const core::Keyframe::SharedPtr& current_keyframe,
        core::IGlobalMapBuilder::SharedPtr global_map) override;

private:
    std::optional<core::Keyframe::SharedPtr> findCandidate(
        const core::Keyframe::SharedPtr& current_keyframe,
        const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const;

    void buildLocalMap(const core::Keyframe::SharedPtr& current_keyframe,
                       const core::Keyframe::SharedPtr& candidate,
                       const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const;

    std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> local_map_;
    std::shared_ptr<scan_matcher::IkdTreeScanMatcher> scan_matcher_;
    IkdTreeLoopClosureDetectorParams params_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_
