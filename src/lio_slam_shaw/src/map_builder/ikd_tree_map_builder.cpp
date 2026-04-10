#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

#include <Eigen/Dense>

namespace lio_slam_shaw::map_builder {

IkdTreeMapBuilder::IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params)
    : params_(params),
      ikd_tree_(params.ikd_delete_param, params.ikd_balance_param, params.ikd_downsample_size) {}

// ── addFrame ──────────────────────────────────────────────────────────────────

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::addFrame(
    const core::LidarFrame::SharedPtr& frame) {
    const auto& cloud_body = frame->features.raw_deskewed;
    if (!cloud_body || cloud_body->empty()) return std::nullopt;

    const Eigen::Isometry3d& T_map_body = frame->matched_result.pose;

    // 1. 將 body-frame 點雲轉到 world frame，插入 ikd-Tree
    KD_TREE<core::PointXYZIRT>::PointVector points_world;
    points_world.reserve(cloud_body->size());
    for (const auto& p : *cloud_body) {
        Eigen::Vector3d pw = T_map_body * Eigen::Vector3d(p.x, p.y, p.z);
        core::PointXYZIRT pt = p;
        pt.x = static_cast<float>(pw.x());
        pt.y = static_cast<float>(pw.y());
        pt.z = static_cast<float>(pw.z());
        points_world.push_back(pt);
    }
    ikd_tree_.Add_Points(points_world, true);  // downsample_on = true，每幀都插入

    // 2. 判斷是否為 keyframe（僅供後端 pose graph 使用，與 ikd-Tree 插入無關）
    if (!isNewKeyframe(T_map_body)) return std::nullopt;

    // 3. 建立 KeyFrame（cloud 保存 body frame，不轉換）
    auto kf = std::make_shared<core::KeyFrame>();
    kf->id = next_keyframe_id_++;
    kf->timestamp = frame->timestamp;
    kf->pose = T_map_body;
    kf->features = frame->features;
    kf->cloud = cloud_body;

    keyframe_index_[kf->id] = keyframes_.size();
    keyframes_.push_back(kf);

    return kf;
}

// ── queryNearestPoints ────────────────────────────────────────────────────────

std::vector<core::NearestPointResult> IkdTreeMapBuilder::queryNearestPoints(
    const core::PointCloudIRTPtr& query_cloud, const Eigen::Isometry3d& T_map_lidar, int k) const {
    std::vector<core::NearestPointResult> results;
    if (!query_cloud || query_cloud->empty()) return results;
    results.reserve(query_cloud->size());

    for (const auto& p : *query_cloud) {
        // 將 query 點轉到 map frame
        Eigen::Vector3d p_map = T_map_lidar * Eigen::Vector3d(p.x, p.y, p.z);
        core::PointXYZIRT query_pt;
        query_pt.x = static_cast<float>(p_map.x());
        query_pt.y = static_cast<float>(p_map.y());
        query_pt.z = static_cast<float>(p_map.z());

        KD_TREE<core::PointXYZIRT>::PointVector neighbors;
        std::vector<float> distances;
        // const_cast 是必要的：ikd-Tree 的 Nearest_Search 內部會拿 mutex，但邏輯上不修改地圖
        const_cast<KD_TREE<core::PointXYZIRT>&>(ikd_tree_).Nearest_Search(
            query_pt, k, neighbors, distances, params_.max_search_dist);

        if (static_cast<int>(neighbors.size()) < params_.min_plane_points) {
            results.push_back({false, {}, {}, {}});
            continue;
        }

        results.push_back(fitPlane(neighbors, p_map));
    }

    return results;
}

// ── updateKeyframePoses ───────────────────────────────────────────────────────

void IkdTreeMapBuilder::updateKeyframePoses(
    const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) {
    // 1. 更新 keyframe poses
    for (const auto& [id, pose] : id_pose_pairs) {
        auto it = keyframe_index_.find(id);
        if (it != keyframe_index_.end()) {
            keyframes_[it->second]->pose = pose;
        }
    }

    // 2. 清空 ikd-Tree，用更新後的 poses 重建
    //    所有 world-frame 座標都過時了，必須全部重建
    KD_TREE<core::PointXYZIRT>::PointVector all_points;
    for (const auto& kf : keyframes_) {
        if (!kf->cloud || kf->cloud->empty()) continue;
        for (const auto& p : *kf->cloud) {
            Eigen::Vector3d pw = kf->pose * Eigen::Vector3d(p.x, p.y, p.z);
            core::PointXYZIRT pt = p;
            pt.x = static_cast<float>(pw.x());
            pt.y = static_cast<float>(pw.y());
            pt.z = static_cast<float>(pw.z());
            all_points.push_back(pt);
        }
    }

    ikd_tree_.~KD_TREE();
    new (&ikd_tree_) KD_TREE<core::PointXYZIRT>(params_.ikd_delete_param, params_.ikd_balance_param,
                                                params_.ikd_downsample_size);
    if (!all_points.empty()) {
        ikd_tree_.Build(all_points);
    }
}

// ── Keyframe 查詢 ─────────────────────────────────────────────────────────────

std::vector<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getAllKeyframes() const {
    return keyframes_;
}

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getKeyframe(uint64_t id) const {
    auto it = keyframe_index_.find(id);
    if (it == keyframe_index_.end()) return std::nullopt;
    return keyframes_[it->second];
}

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getLatestKeyframe() const {
    if (keyframes_.empty()) return std::nullopt;
    return keyframes_.back();
}

core::PointCloudIRTPtr IkdTreeMapBuilder::getGlobalMap() const {
    KD_TREE<core::PointXYZIRT>::PointVector all_points;
    const_cast<KD_TREE<core::PointXYZIRT>&>(ikd_tree_).flatten(ikd_tree_.Root_Node, all_points,
                                                               NOT_RECORD);

    auto cloud = std::make_shared<core::PointCloudIRT>();
    cloud->reserve(all_points.size());
    for (const auto& p : all_points) cloud->push_back(p);
    return cloud;
}

// ── 私有工具函式 ──────────────────────────────────────────────────────────────

bool IkdTreeMapBuilder::isNewKeyframe(const Eigen::Isometry3d& pose) const {
    if (keyframes_.empty()) return true;

    const Eigen::Isometry3d& last_pose = keyframes_.back()->pose;
    double dist = (pose.translation() - last_pose.translation()).norm();
    if (dist >= params_.keyframe_distance_threshold) return true;

    // 旋轉角差：trace(R1^T * R2) = 1 + 2*cos(θ) → θ = acos((trace-1)/2)
    Eigen::Matrix3d dR = last_pose.linear().transpose() * pose.linear();
    double cos_angle = std::clamp((dR.trace() - 1.0) / 2.0, -1.0, 1.0);
    double angle = std::acos(cos_angle);
    return angle >= params_.keyframe_angle_threshold;
}

core::NearestPointResult IkdTreeMapBuilder::fitPlane(
    const KD_TREE<core::PointXYZIRT>::PointVector& neighbors,
    const Eigen::Vector3d& query_point_in_map) const {
    // PCA 平面擬合
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& p : neighbors) centroid += Eigen::Vector3d(p.x, p.y, p.z);
    centroid /= static_cast<double>(neighbors.size());

    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : neighbors) {
        Eigen::Vector3d dp = Eigen::Vector3d(p.x, p.y, p.z) - centroid;
        cov += dp * dp.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    const auto& eigenvalues = solver.eigenvalues();

    // 最小特徵值對應的特徵向量即為平面法向量
    // 有效平面條件：λ₀ << λ₁ ≈ λ₂（薄平面結構）
    if (eigenvalues(0) > params_.plane_valid_threshold * eigenvalues(2)) {
        return {false, {}, {}, {}};
    }

    Eigen::Vector3d normal = solver.eigenvectors().col(0);

    return core::NearestPointResult{
        /*valid=*/true,
        /*point_in_map=*/query_point_in_map,
        /*normal=*/normal,
        /*centroid=*/centroid,
    };
}

}  // namespace lio_slam_shaw::map_builder
