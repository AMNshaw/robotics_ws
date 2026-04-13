#include "lio_slam_shaw/map_optimizer/gtsam_map_optimizer.hpp"

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

namespace lio_slam_shaw {

using gtsam::symbol_shorthand::X;

GtsamMapOptimizer::GtsamMapOptimizer(const GtsamMapOptimizerParams& params) : params_(params) {
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = params_.isam2_relinearize_threshold;
    isam_params.relinearizeSkip = params_.isam2_relinearize_skip;
    isam_params.enableRelinearization = true;
    optimizer_ = std::make_unique<gtsam::ISAM2>(isam_params);

    const auto& pn = params_.prior_noise;
    prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << pn[0], pn[1], pn[2], pn[3], pn[4], pn[5]).finished());

    const auto& on = params_.odom_noise;
    odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << on[0], on[1], on[2], on[3], on[4], on[5]).finished());

    const auto& ond = params_.odom_noise_degenerate;
    odom_noise_degenerate_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << ond[0], ond[1], ond[2], ond[3], ond[4], ond[5]).finished());
}

void GtsamMapOptimizer::addKeyframe(uint64_t keyframe_id, const core::ScanMatchResult& result) {
    std::lock_guard<std::mutex> lock(mtx_);

    const gtsam::Key curr_key = static_cast<gtsam::Key>(id_to_key_.size());
    id_to_key_[keyframe_id] = curr_key;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    const gtsam::Pose3 curr_pose = toGtsamPose(result.pose);

    if (!has_first_keyframe_) {
        graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(curr_key), curr_pose, prior_noise_));
        has_first_keyframe_ = true;
    } else {
        const gtsam::Key prev_key = id_to_key_.at(last_keyframe_id_);
        const gtsam::Pose3 prev_pose = toGtsamPose(id_to_pose_.at(last_keyframe_id_));
        const gtsam::Pose3 relative_pose = prev_pose.between(curr_pose);

        auto noise = result.is_degenerate ? odom_noise_degenerate_ : odom_noise_;
        graph.add(
            gtsam::BetweenFactor<gtsam::Pose3>(X(prev_key), X(curr_key), relative_pose, noise));
    }

    values.insert(X(curr_key), curr_pose);

    optimizer_->update(graph, values);
    optimizer_->update();

    const gtsam::Values estimate = optimizer_->calculateBestEstimate();
    for (const auto& [id, key] : id_to_key_) {
        if (estimate.exists(X(key))) {
            id_to_pose_[id] = fromGtsamPose(estimate.at<gtsam::Pose3>(X(key)));
        }
    }

    last_keyframe_id_ = keyframe_id;
}

void GtsamMapOptimizer::addLoopConstraint(const core::LoopConstraint& constraint) {
    std::lock_guard<std::mutex> lock(mtx_);

    const auto from_it = id_to_key_.find(constraint.from_id);
    const auto to_it = id_to_key_.find(constraint.to_id);
    if (from_it == id_to_key_.end() || to_it == id_to_key_.end()) return;

    const gtsam::Key from_key = from_it->second;
    const gtsam::Key to_key = to_it->second;

    auto noise = gtsam::noiseModel::Gaussian::Covariance(constraint.covariance);

    loop_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        X(from_key), X(to_key), toGtsamPose(constraint.relative_pose), noise));
}

std::vector<std::pair<uint64_t, Eigen::Isometry3d>> GtsamMapOptimizer::optimize() {
    std::lock_guard<std::mutex> lock(mtx_);

    optimizer_->update(loop_graph_, gtsam::Values());
    loop_graph_.resize(0);

    for (int i = 0; i < 3; ++i) {
        optimizer_->update();
    }

    const gtsam::Values estimate = optimizer_->calculateBestEstimate();
    for (const auto& [id, key] : id_to_key_) {
        if (estimate.exists(X(key))) {
            id_to_pose_[id] = fromGtsamPose(estimate.at<gtsam::Pose3>(X(key)));
        }
    }

    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> result;
    result.reserve(id_to_pose_.size());
    for (const auto& [id, pose] : id_to_pose_) {
        result.emplace_back(id, pose);
    }
    return result;
}

core::NavState GtsamMapOptimizer::getKeyframePose(uint64_t keyframe_id) const {
    std::lock_guard<std::mutex> lock(mtx_);
    core::NavState state;
    const auto it = id_to_pose_.find(keyframe_id);
    if (it != id_to_pose_.end()) {
        state.pose = it->second;
    }
    return state;
}

gtsam::Pose3 GtsamMapOptimizer::toGtsamPose(const Eigen::Isometry3d& pose) {
    return gtsam::Pose3(pose.matrix());
}

Eigen::Isometry3d GtsamMapOptimizer::fromGtsamPose(const gtsam::Pose3& pose) {
    return Eigen::Isometry3d(pose.matrix());
}

}  // namespace lio_slam_shaw
