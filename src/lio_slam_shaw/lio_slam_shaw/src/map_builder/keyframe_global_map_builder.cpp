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

size_t KeyframeGlobalMapBuilder::getKeyframeCount() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    return keyframes_.size();
}

core::PointCloudIRTPtr KeyframeGlobalMapBuilder::getGlobalMap() const {
    // Snapshot keyframe data under lock, then assemble outside to avoid blocking addKeyFrame
    struct KfSnapshot {
        Eigen::Isometry3d pose;
        core::PointCloudIRTConstPtr cloud_body;
    };
    std::vector<KfSnapshot> snapshots;
    {
        std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
        snapshots.reserve(keyframes_.size());
        for (const auto& kf : keyframes_) {
            snapshots.push_back({kf->pose, kf->cloud_body});
        }
    }

    const int n_kf = static_cast<int>(snapshots.size());
    auto cloud = std::make_shared<core::PointCloudIRT>();

    // Compute prefix offsets for parallel write
    std::vector<size_t> offsets(n_kf + 1, 0);
    for (int i = 0; i < n_kf; ++i) {
        offsets[i + 1] =
            offsets[i] + (snapshots[i].cloud_body ? snapshots[i].cloud_body->size() : 0);
    }
    cloud->resize(offsets[n_kf]);

#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < n_kf; ++i) {
        const auto& snap = snapshots[i];
        if (!snap.cloud_body || snap.cloud_body->empty()) continue;
        const Eigen::Isometry3d T = snap.pose * params_.T_base_lidar;
        size_t offset = offsets[i];
        for (size_t j = 0; j < snap.cloud_body->size(); ++j) {
            const auto& p = (*snap.cloud_body)[j];
            Eigen::Vector3d pw = T * Eigen::Vector3d(p.x, p.y, p.z);
            core::PointXYZIRT pt = p;
            pt.x = static_cast<float>(pw.x());
            pt.y = static_cast<float>(pw.y());
            pt.z = static_cast<float>(pw.z());
            (*cloud)[offset + j] = pt;
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
