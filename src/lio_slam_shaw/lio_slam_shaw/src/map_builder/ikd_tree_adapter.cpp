#include "lio_slam_shaw/map_builder/ikd_tree_adapter.hpp"

#include <omp.h>

namespace lio_slam_shaw::map_builder {

IkdTreeAdapter::IkdTreeAdapter(const Config& params) : params_(params) {
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params_.ikd_delete_param, params_.ikd_balance_param, params_.ikd_downsample_size);
}

void IkdTreeAdapter::addScan(const Eigen::Isometry3d& T_map_sensor,
                             const core::PointCloudIRTConstPtr& cloud_sensor) {
    if (!cloud_sensor || cloud_sensor->empty()) return;
    size_t n_points = cloud_sensor->size();

    KD_TREE<core::PointXYZIRT>::PointVector points_world(n_points);

#pragma omp parallel for num_threads(4) schedule(dynamic)
    for (int i = 0; i < n_points; ++i) {
        const auto& p = (*cloud_sensor)[i];
        // TODO: Unroll matrix multiplication to element-wise operations
        Eigen::Vector3d pw = T_map_sensor * Eigen::Vector3d(p.x, p.y, p.z);
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
}

void IkdTreeAdapter::boxTrim(const Eigen::Vector3d& center, double half_length) {
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

void IkdTreeAdapter::clearMap() {
    std::unique_lock<std::shared_mutex> lock(ikd_tree_mutex_);
    ikd_tree_.reset();
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params_.ikd_delete_param, params_.ikd_balance_param, params_.ikd_downsample_size);
    is_first_frame_ = true;
}

bool IkdTreeAdapter::searchNearest(const core::PointXYZIRT& query_pt, int k, float search_dist,
                                   const PointVector*& out_neighbors,
                                   const std::vector<float>*& out_distances) const {
    if (!ikd_tree_) return false;

    thread_local PointVector tls_neighbors;
    thread_local std::vector<float> tls_distances;

    const_cast<KD_TREE<core::PointXYZIRT>*>(ikd_tree_.get())
        ->Nearest_Search(query_pt, k, tls_neighbors, tls_distances, search_dist);

    out_neighbors = &tls_neighbors;
    out_distances = &tls_distances;
    return true;
}

NearestPlaneResult IkdTreeAdapter::queryNearestPlane(const core::PointXYZIRT& query_pt,
                                                     float search_dist) {
    const PointVector* neighbors_ptr = nullptr;
    const std::vector<float>* distances_ptr = nullptr;
    if (!searchNearest(query_pt, params_.k_query_neighbors, search_dist, neighbors_ptr,
                       distances_ptr))
        return NearestPlaneResult{};

    if (static_cast<int>(neighbors_ptr->size()) < params_.min_plane_points) {
        return NearestPlaneResult{};
    }

    return fitPlane(*neighbors_ptr, Eigen::Vector3d(query_pt.x, query_pt.y, query_pt.z));
}

NearestPlaneResult IkdTreeAdapter::fitPlane(const PointVector& neighbors,
                                            const Eigen::Vector3d& query_point_in_map) const {
    NearestPlaneResult result;
    result.point_in_map = query_point_in_map;

    result.centroid = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) result.centroid += Eigen::Vector3d(p.x, p.y, p.z);
    result.centroid /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d dp = Eigen::Vector3d(p.x, p.y, p.z) - result.centroid;
        cov += dp * dp.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const auto& eigenvalues = solver.eigenvalues();

    if (eigenvalues(0) > params_.plane_valid_threshold * eigenvalues(2)) {
        return {false, {}, {}, {}};
    }

    result.normal = solver.eigenvectors().col(0);

    return result;
}

}  // namespace lio_slam_shaw::map_builder