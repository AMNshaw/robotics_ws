#include "lio_slam_shaw/map_builder/keyframe_global_map_builder.hpp"

#include <omp.h>

#include <Eigen/Dense>
#include <cmath>

namespace lio_slam_shaw::map_builder {

KeyframeGlobalMapBuilder::KeyframeGlobalMapBuilder(const KeyframeGlobalMapBuilderParams& params)
    : params_(params) {}

std::optional<core::Keyframe::SharedPtr> KeyframeGlobalMapBuilder::addKeyFrame(
    const core::LidarFrame::SharedPtr& frame) {
    const Eigen::Isometry3d& T_map_body = frame->matched_result.pose;

    if (!isNewKeyframe(T_map_body)) return std::nullopt;

    const auto& cloud_body = frame->features.raw_deskewed;

    auto kf = std::make_shared<core::Keyframe>(next_keyframe_id_++, frame->timestamp, T_map_body,
                                               frame->features, cloud_body, frame->matched_result);

    std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
    keyframe_index_[kf->id] = keyframes_.size();
    keyframes_.push_back(kf);
    return kf;
}

void KeyframeGlobalMapBuilder::updateKeyframePoses(
    const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) {
    std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
    for (const auto& [id, pose] : id_pose_pairs) {
        auto it = keyframe_index_.find(id);
        if (it != keyframe_index_.end()) {
            keyframes_[it->second]->pose = pose;
        }
    }
}

std::vector<core::Keyframe::SharedPtr> KeyframeGlobalMapBuilder::getAllKeyframes() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    return keyframes_;
}

std::optional<core::Keyframe::SharedPtr> KeyframeGlobalMapBuilder::getKeyframe(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    auto it = keyframe_index_.find(id);
    if (it == keyframe_index_.end()) return std::nullopt;
    return keyframes_[it->second];
}

std::optional<core::Keyframe::SharedPtr> KeyframeGlobalMapBuilder::getLatestKeyframe() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    if (keyframes_.empty()) return std::nullopt;
    return keyframes_.back();
}

core::PointCloudIRTPtr KeyframeGlobalMapBuilder::getGlobalMap() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    auto cloud = std::make_shared<core::PointCloudIRT>();

    size_t total = 0;
    for (const auto& kf : keyframes_) {
        if (kf->cloud_body) total += kf->cloud_body->size();
    }
    cloud->reserve(total);

    for (const auto& kf : keyframes_) {
        if (!kf->cloud_body || kf->cloud_body->empty()) continue;
        const Eigen::Isometry3d T = kf->pose * params_.T_base_lidar;
        for (const auto& p : *kf->cloud_body) {
            Eigen::Vector3d pw = T * Eigen::Vector3d(p.x, p.y, p.z);
            core::PointXYZIRT pt = p;
            pt.x = static_cast<float>(pw.x());
            pt.y = static_cast<float>(pw.y());
            pt.z = static_cast<float>(pw.z());
            cloud->push_back(pt);
        }
    }
    return cloud;
}

bool KeyframeGlobalMapBuilder::isNewKeyframe(const Eigen::Isometry3d& pose) const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    if (keyframes_.empty()) return true;

    const Eigen::Isometry3d& last_pose = keyframes_.back()->pose;
    double dist = (pose.translation() - last_pose.translation()).norm();
    if (dist >= params_.keyframe_distance_threshold) return true;

    Eigen::Matrix3d dR = last_pose.linear().transpose() * pose.linear();
    double cos_angle = std::clamp((dR.trace() - 1.0) / 2.0, -1.0, 1.0);
    double angle = std::acos(cos_angle);
    return angle >= params_.keyframe_angle_threshold;
}

}  // namespace lio_slam_shaw::map_builder
