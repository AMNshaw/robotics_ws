#ifndef LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_
#define LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_

#include <memory>
#include <optional>

#include "lio_slam_shaw/core/i_loop_closure_detector.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw {

struct IkdTreeLoopClosureDetectorParams {
    // 第一層：候選篩選
    double search_radius = 15.0;
    double min_time_gap_sec = 30.0;
    int local_map_keyframe_num = 25;

    double fitness_score_threshold = 0.3;  // 接受 loop 的 fitness 門檻（越小越嚴格）
};

// Loop closure detector，直接委託給 IScanMatcher 做 point-to-plane 配準
// 不重複 GN 邏輯，covariance 也直接取自 scan_matcher 回傳的 H^{-1}
class IkdTreeLoopClosureDetector : public core::ILoopClosureDetector {
public:
    IkdTreeLoopClosureDetector(const scan_matcher::IkdTreeScanMatcherParams& scan_matcher_params,
                               const IkdTreeLoopClosureDetectorParams& loop_closure_params);
    ~IkdTreeLoopClosureDetector() override = default;

    std::optional<core::LoopConstraint> detect(const core::KeyFrame::SharedPtr& current_keyframe,
                                               core::IMapBuilder::SharedPtr map_builder) override;

private:
    std::optional<core::KeyFrame::SharedPtr> findCandidate(
        const core::KeyFrame::SharedPtr& current_keyframe,
        const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const;

    void buildLocalMap(const core::KeyFrame::SharedPtr& current_keyframe,
                       const core::KeyFrame::SharedPtr& candidate,
                       const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const;

    std::shared_ptr<map_builder::IkdTreeMapBuilder> map_builder_;
    std::shared_ptr<scan_matcher::IkdTreeScanMatcher> scan_matcher_;
    IkdTreeLoopClosureDetectorParams params_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__LOOP_CLOSURE__IKD_TREE_LOOP_CLOSURE_DETECTOR_HPP_
