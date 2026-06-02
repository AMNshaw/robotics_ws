#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"

#include <omp.h>

#include <Eigen/Dense>

#include "lio_slam_shaw/core/keyframe.hpp"

namespace lio_slam_shaw::map_builder {

IkdTreeLocalMapBuilder::IkdTreeLocalMapBuilder(const IkdTreeLocalMapBuilderParams& params)
    : params_(params) {
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params.ikd_delete_param, params.ikd_balance_param, params.ikd_downsample_size);
}

void IkdTreeLocalMapBuilder::addScan(const core::LidarFrame::SharedPtr& frame) {
    const auto& cloud_body = frame->features.raw_deskewed;
    if (!cloud_body || cloud_body->empty()) return;

    const Eigen::Isometry3d& T_map_body = frame->matched_result.pose;
    const Eigen::Isometry3d T_map_lidar = T_map_body * params_.T_base_lidar;

    size_t n_points = cloud_body->size();
    KD_TREE<core::PointXYZIRT>::PointVector points_world(n_points);

#pragma omp parallel for num_threads(4) schedule(dynamic)
    for (size_t i = 0; i < n_points; ++i) {
        const auto& p = (*cloud_body)[i];
        Eigen::Vector3d pw = T_map_lidar * Eigen::Vector3d(p.x, p.y, p.z);
        points_world[i] = p;
        points_world[i].x = static_cast<float>(pw.x());
        points_world[i].y = static_cast<float>(pw.y());
        points_world[i].z = static_cast<float>(pw.z());
    }

    std::unique_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    if (is_first_frame_) {
        ikd_tree_->Build(points_world);
        is_first_frame_ = false;
    } else {
        ikd_tree_->Add_Points(points_world, true);
    }
    lock.unlock();

    // Box trim around current position
    const Eigen::Vector3d pos = frame->state_odom.pose.translation();
    boxTrim(pos, params_.box_trim_half_length);
}

bool IkdTreeLocalMapBuilder::isMapReady() const {
    std::shared_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    return !is_first_frame_;
}

void IkdTreeLocalMapBuilder::boxTrim(const Eigen::Vector3d& center, double half_length) {
    BoxPointType box;
    box.vertex_min[0] = center.x() - half_length;
    box.vertex_min[1] = center.y() - half_length;
    box.vertex_min[2] = center.z() - half_length;
    box.vertex_max[0] = center.x() + half_length;
    box.vertex_max[1] = center.y() + half_length;
    box.vertex_max[2] = center.z() + half_length;

    std::unique_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    if (!ikd_tree_ || is_first_frame_) return;

    // Delete points OUTSIDE the box: create 6 slabs covering the exterior
    std::vector<BoxPointType> delete_boxes;
    delete_boxes.reserve(6);
    constexpr float INF = 1e6f;

    // -X slab
    BoxPointType b;
    b.vertex_min[0] = -INF;
    b.vertex_min[1] = -INF;
    b.vertex_min[2] = -INF;
    b.vertex_max[0] = box.vertex_min[0];
    b.vertex_max[1] = INF;
    b.vertex_max[2] = INF;
    delete_boxes.push_back(b);
    // +X slab
    b.vertex_min[0] = box.vertex_max[0];
    b.vertex_min[1] = -INF;
    b.vertex_min[2] = -INF;
    b.vertex_max[0] = INF;
    b.vertex_max[1] = INF;
    b.vertex_max[2] = INF;
    delete_boxes.push_back(b);
    // -Y slab (within X bounds)
    b.vertex_min[0] = box.vertex_min[0];
    b.vertex_min[1] = -INF;
    b.vertex_min[2] = -INF;
    b.vertex_max[0] = box.vertex_max[0];
    b.vertex_max[1] = box.vertex_min[1];
    b.vertex_max[2] = INF;
    delete_boxes.push_back(b);
    // +Y slab (within X bounds)
    b.vertex_min[0] = box.vertex_min[0];
    b.vertex_min[1] = box.vertex_max[1];
    b.vertex_min[2] = -INF;
    b.vertex_max[0] = box.vertex_max[0];
    b.vertex_max[1] = INF;
    b.vertex_max[2] = INF;
    delete_boxes.push_back(b);
    // -Z slab (within XY bounds)
    b.vertex_min[0] = box.vertex_min[0];
    b.vertex_min[1] = box.vertex_min[1];
    b.vertex_min[2] = -INF;
    b.vertex_max[0] = box.vertex_max[0];
    b.vertex_max[1] = box.vertex_max[1];
    b.vertex_max[2] = box.vertex_min[2];
    delete_boxes.push_back(b);
    // +Z slab (within XY bounds)
    b.vertex_min[0] = box.vertex_min[0];
    b.vertex_min[1] = box.vertex_min[1];
    b.vertex_min[2] = box.vertex_max[2];
    b.vertex_max[0] = box.vertex_max[0];
    b.vertex_max[1] = box.vertex_max[1];
    b.vertex_max[2] = INF;
    delete_boxes.push_back(b);

    ikd_tree_->Delete_Point_Boxes(delete_boxes);
}

void IkdTreeLocalMapBuilder::clearMap() {
    std::unique_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    ikd_tree_.reset();
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params_.ikd_delete_param, params_.ikd_balance_param, params_.ikd_downsample_size);
    is_first_frame_ = true;
}

void IkdTreeLocalMapBuilder::addKeyFrame(const core::Keyframe::SharedPtr& keyframe) {
    if (!keyframe || !keyframe->cloud_body || keyframe->cloud_body->empty()) return;

    const Eigen::Isometry3d T_map_lidar = keyframe->pose * params_.T_base_lidar;
    size_t n_points = keyframe->cloud_body->size();
    KD_TREE<core::PointXYZIRT>::PointVector points_world(n_points);

#pragma omp parallel for num_threads(4) schedule(dynamic)
    for (size_t i = 0; i < n_points; ++i) {
        const auto& p = (*keyframe->cloud_body)[i];
        Eigen::Vector3d pw = T_map_lidar * Eigen::Vector3d(p.x, p.y, p.z);
        points_world[i] = p;
        points_world[i].x = static_cast<float>(pw.x());
        points_world[i].y = static_cast<float>(pw.y());
        points_world[i].z = static_cast<float>(pw.z());
    }

    std::unique_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    if (is_first_frame_) {
        ikd_tree_->Build(points_world);
        is_first_frame_ = false;
    } else {
        ikd_tree_->Add_Points(points_world, true);
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// IkdTreeLocalReadSession
// ─────────────────────────────────────────────────────────────────────────────

IkdTreeLocalReadSession::IkdTreeLocalReadSession(const IkdTreeLocalMapBuilder& owner)
    : owner_(&owner), lock_(owner.ikd_tree_mutex_) {}

bool IkdTreeLocalReadSession::searchKNearest(const core::PointXYZIRT& query_pt, int k,
                                             float search_dist, const PointVector*& out_neighbors,
                                             const std::vector<float>*& out_distances) const {
    if (!owner_ || !owner_->ikd_tree_) return false;

    thread_local PointVector tls_neighbors;
    thread_local std::vector<float> tls_distances;

    const_cast<KD_TREE<core::PointXYZIRT>*>(owner_->ikd_tree_.get())
        ->Nearest_Search(query_pt, k, tls_neighbors, tls_distances, search_dist);

    out_neighbors = &tls_neighbors;
    out_distances = &tls_distances;
    return true;
}

}  // namespace lio_slam_shaw::map_builder
