#include "lio_slam_shaw/loop_closure_detector/ikd_tree_loop_closure_detector.hpp"

#include <algorithm>
#include <limits>

namespace lio_slam_shaw {

IkdTreeLoopClosureDetector::IkdTreeLoopClosureDetector(
    core::IScanMatcher::SharedPtr scan_matcher, const IkdTreeLoopClosureDetectorParams& params)
    : scan_matcher_(std::move(scan_matcher)), params_(params) {}

// ─────────────────────────────────────────────────────────────────────────────
// detect
// ─────────────────────────────────────────────────────────────────────────────
std::optional<core::LoopConstraint> IkdTreeLoopClosureDetector::detect(
    const core::KeyFrame::SharedPtr& current_keyframe, core::IMapBuilder::SharedPtr map_builder) {
    const auto all_keyframes = map_builder->getAllKeyframes();
    if (all_keyframes.size() < 2) return std::nullopt;

    // ── 第一層：找候選 ──────────────────────────────────────────────────────
    const auto candidate_opt = findCandidate(current_keyframe, all_keyframes);
    if (!candidate_opt.has_value()) return std::nullopt;
    const auto& candidate = candidate_opt.value();

    if (!current_keyframe->cloud || current_keyframe->cloud->empty()) return std::nullopt;

    // ── 第二層：委託給 scan_matcher 做 point-to-plane 配準 ─────────────────
    // initial guess：用 current keyframe 的已知 pose（在 global map frame）
    core::NavState initial_guess;
    initial_guess.pose = current_keyframe->pose;

    // scan_matcher 使用的是 ikd-Tree（global map），直接查最近鄰做 GN
    // features.raw_deskewed 是 body frame 的點雲
    core::FeatureSet features;
    features.raw_deskewed = current_keyframe->cloud;

    const auto result = scan_matcher_->match(features, initial_guess);
    if (!result.is_converged) return std::nullopt;

    // ── 計算相對位姿 ──────────────────────────────────────────────────────
    // result.pose = T_w_from_corrected（GN 最佳化後的 current frame 位姿）
    // BetweenFactor(from, to, z): z = T_w_from_corrected.inverse() * T_w_to
    const Eigen::Isometry3d relative_pose = result.pose.inverse() * candidate->pose;

    // covariance 直接取 scan_matcher 計算的 H^{-1}，比固定 sigma 更準確
    return core::LoopConstraint{current_keyframe->id, candidate->id, relative_pose,
                                result.covariance, result.fitness_score};
}

// ─────────────────────────────────────────────────────────────────────────────
// findCandidate — radius search + time gap filter
// ─────────────────────────────────────────────────────────────────────────────
std::optional<core::KeyFrame::SharedPtr> IkdTreeLoopClosureDetector::findCandidate(
    const core::KeyFrame::SharedPtr& current_keyframe,
    const std::vector<core::KeyFrame::SharedPtr>& all_keyframes) const {
    const Eigen::Vector3d curr_pos = current_keyframe->pose.translation();
    core::KeyFrame::SharedPtr best = nullptr;
    double best_dist = std::numeric_limits<double>::max();

    for (const auto& kf : all_keyframes) {
        if (kf->id == current_keyframe->id) continue;

        const double time_diff =
            std::abs(core::getDeltaSec(kf->timestamp, current_keyframe->timestamp));
        if (time_diff < params_.min_time_gap_sec) continue;

        const double dist = (kf->pose.translation() - curr_pos).norm();
        if (dist > params_.search_radius) continue;

        if (dist < best_dist) {
            best_dist = dist;
            best = kf;
        }
    }

    if (!best) return std::nullopt;
    return best;
}

}  // namespace lio_slam_shaw
