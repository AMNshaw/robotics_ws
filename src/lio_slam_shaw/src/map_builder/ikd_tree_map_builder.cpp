#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

#include <omp.h>

#include <Eigen/Dense>

namespace lio_slam_shaw::map_builder {

IkdTreeMapBuilder::IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params) : params_(params) {
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params.ikd_delete_param, params.ikd_balance_param, params.ikd_downsample_size);
}

std::optional<core::Keyframe::SharedPtr> IkdTreeMapBuilder::addFrame(
    const core::LidarFrame::SharedPtr& frame) {
    const auto& cloud_body = frame->features.raw_deskewed;
    if (!cloud_body || cloud_body->empty()) return std::nullopt;

    const Eigen::Isometry3d& T_map_body = frame->matched_result.pose;

    size_t n_points = cloud_body->size();
    KD_TREE<core::PointXYZIRT>::PointVector points_world(n_points);

#pragma omp parallel for num_threads(4) schedule(dynamic)
    for (size_t i = 0; i < n_points; ++i) {
        const auto& p = (*cloud_body)[i];
        Eigen::Vector3d pw = T_map_body * Eigen::Vector3d(p.x, p.y, p.z);

        points_world[i] = p;
        points_world[i].x = static_cast<float>(pw.x());
        points_world[i].y = static_cast<float>(pw.y());
        points_world[i].z = static_cast<float>(pw.z());
    }
    auto local_tree = ikd_tree_;
    local_tree->Add_Points(points_world, true);

    if (is_updating_map_.load()) {
        KD_TREE<core::PointXYZIRT>::PointVector points_corrected = points_world;

        std::optional<Eigen::Isometry3d> T_corr;
        {
            std::shared_lock<std::shared_mutex> lock(correction_mutex_);
            T_corr = last_pending_correction_;
        }

        if (T_corr.has_value()) {
            const Eigen::Isometry3d& mat = T_corr.value();
#pragma omp parallel for num_threads(4) schedule(static)
            for (size_t i = 0; i < points_corrected.size(); ++i) {
                auto& p = points_corrected[i];
                Eigen::Vector3d pt_new = mat * Eigen::Vector3d(p.x, p.y, p.z);
                p.x = pt_new.x();
                p.y = pt_new.y();
                p.z = pt_new.z();
            }
        }

        std::lock_guard<std::mutex> lock(increment_mutex_);
        ikd_increment_.insert(ikd_increment_.end(),
                              std::make_move_iterator(points_corrected.begin()),
                              std::make_move_iterator(points_corrected.end()));
    }

    if (!isNewKeyframe(T_map_body)) return std::nullopt;

    {
        std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
        auto kf =
            std::make_shared<core::Keyframe>(next_keyframe_id_++, frame->timestamp, T_map_body,
                                             frame->features, cloud_body, frame->matched_result);

        keyframe_index_[kf->id] = keyframes_.size();
        keyframes_.push_back(kf);
        return kf;
    }
}

void IkdTreeMapBuilder::addKeyFrame(const core::Keyframe::SharedPtr& keyframe) {
    if (!keyframe || !keyframe->cloud_body || keyframe->cloud_body->empty()) return;

    KD_TREE<core::PointXYZIRT>::PointVector points_world;
    points_world.reserve(keyframe->cloud_body->size());
    for (const auto& p : *keyframe->cloud_body) {
        Eigen::Vector3d pw = keyframe->pose * Eigen::Vector3d(p.x, p.y, p.z);
        core::PointXYZIRT pt = p;
        pt.x = static_cast<float>(pw.x());
        pt.y = static_cast<float>(pw.y());
        pt.z = static_cast<float>(pw.z());
        points_world.push_back(pt);
    }
    ikd_tree_->Add_Points(points_world, true);

    {
        std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
        keyframe_index_[keyframe->id] = keyframes_.size();
        keyframes_.push_back(keyframe);
    }
}

void IkdTreeMapBuilder::clearMap() {
    ikd_tree_.reset();
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params_.ikd_delete_param, params_.ikd_balance_param, params_.ikd_downsample_size);
    {
        std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
        keyframes_.clear();
        keyframe_index_.clear();
    }
    next_keyframe_id_ = 0;
}

bool IkdTreeMapBuilder::searchKNearestPoints(const core::PointXYZIRT& query_pt, int k,
                                             float search_dist,
                                             std::vector<core::PointXYZIRT>& out_neighbors,
                                             std::vector<float>& out_distances) const {
    if (!ikd_tree_) return false;

    KD_TREE<core::PointXYZIRT>::PointVector internal_neighbors;

    const_cast<KD_TREE<core::PointXYZIRT>*>(ikd_tree_.get())
        ->Nearest_Search(query_pt, k, internal_neighbors, out_distances, search_dist);

    out_neighbors.assign(internal_neighbors.begin(), internal_neighbors.end());

    return true;
}

void IkdTreeMapBuilder::updateKeyframePoses(
    const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) {
    {
        std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
        if (!id_pose_pairs.empty()) {
            const auto& [last_id, new_pose] = id_pose_pairs.back();
            auto it = keyframe_index_.find(last_id);
            if (it != keyframe_index_.end()) {
                auto old_pose = keyframes_[it->second]->pose;
                {
                    std::unique_lock<std::shared_mutex> lock(correction_mutex_);
                    last_pending_correction_ = new_pose * old_pose.inverse();
                }
            }
        }
    }

    {
        std::lock_guard<std::mutex> lock(increment_mutex_);
        ikd_increment_.clear();
        is_updating_map_.store(true, std::memory_order_release);
    }

    {
        std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
        for (const auto& [id, pose] : id_pose_pairs) {
            auto it = keyframe_index_.find(id);
            if (it != keyframe_index_.end()) {
                keyframes_[it->second]->pose = pose;
            }
        }
    }

    KD_TREE<core::PointXYZIRT>::PointVector all_points;
    size_t total_points = 0;
    {
        std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
        for (const auto& kf : keyframes_) {
            if (kf->cloud_body) total_points += kf->cloud_body->size();
        }
        all_points.resize(total_points);

        size_t current_pos = 0;
        for (const auto& kf : keyframes_) {
            const auto& cloud = kf->cloud_body;
            if (!cloud || cloud->empty()) continue;
            const auto& T = kf->pose;
            size_t kf_size = cloud->size();
#pragma omp parallel for num_threads(4)
            for (size_t i = 0; i < kf_size; ++i) {
                const auto& p = (*cloud)[i];
                Eigen::Vector3d pw = T * Eigen::Vector3d(p.x, p.y, p.z);
                auto& pt_out = all_points[current_pos + i];
                pt_out = p;
                pt_out.x = pw.x();
                pt_out.y = pw.y();
                pt_out.z = pw.z();
            }
            current_pos += kf_size;
        }
    }

    temp_ikd_tree_.reset();
    temp_ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params_.ikd_delete_param, params_.ikd_balance_param, params_.ikd_downsample_size);

    if (!all_points.empty()) {
        temp_ikd_tree_->Build(all_points);
    }

    {
        KD_TREE<core::PointXYZIRT>::PointVector catch_up_points;
        {
            std::lock_guard<std::mutex> lock(increment_mutex_);
            catch_up_points.swap(ikd_increment_);
        }
        if (!catch_up_points.empty()) {
            temp_ikd_tree_->Add_Points(catch_up_points, true);
        }
    }
}

void IkdTreeMapBuilder::updateMap() {
    if (!temp_ikd_tree_) return;

    {
        std::lock_guard<std::mutex> lock(increment_mutex_);
        if (!ikd_increment_.empty()) {
            temp_ikd_tree_->Add_Points(ikd_increment_, true);
            ikd_increment_.clear();
        }
        std::swap(ikd_tree_, temp_ikd_tree_);
    }

    is_updating_map_.store(false, std::memory_order_release);
    last_pending_correction_.reset();
    temp_ikd_tree_.reset();
}

std::vector<core::Keyframe::SharedPtr> IkdTreeMapBuilder::getAllKeyframes() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    return keyframes_;
}

std::optional<core::Keyframe::SharedPtr> IkdTreeMapBuilder::getKeyframe(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    auto it = keyframe_index_.find(id);
    if (it == keyframe_index_.end()) return std::nullopt;
    return keyframes_[it->second];
}

std::optional<core::Keyframe::SharedPtr> IkdTreeMapBuilder::getLatestKeyframe() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    if (keyframes_.empty()) return std::nullopt;
    return keyframes_.back();
}

core::PointCloudIRTPtr IkdTreeMapBuilder::getGlobalMap() const {
    auto local_tree = ikd_tree_;
    if (!local_tree || local_tree->Root_Node == nullptr)
        return std::make_shared<core::PointCloudIRT>();

    KD_TREE<core::PointXYZIRT>::PointVector all_points;
    const_cast<KD_TREE<core::PointXYZIRT>*>(local_tree.get())
        ->flatten(local_tree->Root_Node, all_points, NOT_RECORD);

    auto cloud = std::make_shared<core::PointCloudIRT>();
    cloud->reserve(all_points.size());
    for (const auto& p : all_points) cloud->push_back(p);
    return cloud;
}

bool IkdTreeMapBuilder::isNewKeyframe(const Eigen::Isometry3d& pose) const {
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
