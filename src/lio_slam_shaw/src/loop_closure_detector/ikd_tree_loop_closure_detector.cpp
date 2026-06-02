#include "lio_slam_shaw/loop_closure_detector/ikd_tree_loop_closure_detector.hpp"

#include <algorithm>
#include <limits>

namespace lio_slam_shaw {

IkdTreeLoopClosureDetector::IkdTreeLoopClosureDetector(
    const scan_matcher::IkdTreeScanMatcherParams& scan_matcher_params,
    const IkdTreeLoopClosureDetectorParams& loop_closure_params)
    : params_(loop_closure_params) {
    local_map_ = std::make_shared<map_builder::IkdTreeLocalMapBuilder>(
        map_builder::IkdTreeLocalMapBuilderParams{});
    scan_matcher_ =
        std::make_shared<scan_matcher::IkdTreeScanMatcher>(local_map_, scan_matcher_params);
}

std::optional<core::LoopConstraint> IkdTreeLoopClosureDetector::detect(
    const core::Keyframe::SharedPtr& current_keyframe,
    core::IGlobalMapBuilder::SharedPtr global_map) {
    const auto all_keyframes = global_map->getAllKeyframes();
    if (all_keyframes.size() < 2) return std::nullopt;

    const auto candidate_opt = findCandidate(current_keyframe, all_keyframes);
    if (!candidate_opt.has_value()) return std::nullopt;
    const auto& candidate = candidate_opt.value();

    if (!current_keyframe->cloud_body || current_keyframe->cloud_body->empty()) return std::nullopt;

    buildLocalMap(current_keyframe, candidate, all_keyframes);

    core::NavState initial_guess;
    initial_guess.pose = current_keyframe->pose;

    core::FeatureSet features;
    features.raw_deskewed = current_keyframe->cloud_body;

    const auto result = scan_matcher_->match(features, initial_guess);
    if (!result.is_converged || result.fitness_score > params_.fitness_score_threshold) {
        return std::nullopt;
    }

    const Eigen::Isometry3d relative_pose = result.pose.inverse() * candidate->pose;

    return core::LoopConstraint{current_keyframe->id, candidate->id, relative_pose,
                                result.covariance, result.fitness_score};
}

std::optional<core::Keyframe::SharedPtr> IkdTreeLoopClosureDetector::findCandidate(
    const core::Keyframe::SharedPtr& current_keyframe,
    const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const {
    const Eigen::Vector3d curr_pos = current_keyframe->pose.translation();
    core::Keyframe::SharedPtr best = nullptr;
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

void IkdTreeLoopClosureDetector::buildLocalMap(
    const core::Keyframe::SharedPtr& current_keyframe, const core::Keyframe::SharedPtr& candidate,
    const std::vector<core::Keyframe::SharedPtr>& all_keyframes) const {
    local_map_->clearMap();
    const Eigen::Vector3d cand_pos = candidate->pose.translation();

    std::vector<std::pair<double, core::Keyframe::SharedPtr>> dist_kf;
    dist_kf.reserve(all_keyframes.size());
    for (const auto& kf : all_keyframes) {
        const double time_diff_to_curr =
            std::abs(core::getDeltaSec(kf->timestamp, current_keyframe->timestamp));
        if (time_diff_to_curr < params_.min_time_gap_sec) {
            continue;
        }
        const double d = (kf->pose.translation() - cand_pos).norm();
        dist_kf.emplace_back(d, kf);
    }
    std::sort(dist_kf.begin(), dist_kf.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    const int n = std::min(static_cast<int>(dist_kf.size()), params_.local_map_keyframe_num);

    for (int i = 0; i < n; ++i) {
        const auto& kf = dist_kf[i].second;
        if (!kf->cloud_body || kf->cloud_body->empty()) continue;
        local_map_->addKeyFrame(kf);
    }
}
}  // namespace lio_slam_shaw
