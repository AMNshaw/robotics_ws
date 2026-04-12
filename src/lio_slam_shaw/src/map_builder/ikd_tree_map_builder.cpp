#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"

#include <omp.h>

#include <Eigen/Dense>

namespace lio_slam_shaw::map_builder {

IkdTreeMapBuilder::IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params) : params_(params) {
    ikd_tree_ = std::make_shared<KD_TREE<core::PointXYZIRT>>(
        params.ikd_delete_param, params.ikd_balance_param, params.ikd_downsample_size);
}

// ── addFrame ──────────────────────────────────────────────────────────────────

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::addFrame(
    const core::LidarFrame::SharedPtr& frame) {
    const auto& cloud_body = frame->features.raw_deskewed;
    if (!cloud_body || cloud_body->empty()) return std::nullopt;

    const Eigen::Isometry3d& T_map_body = frame->matched_result.pose;

    // 1. 將 body-frame 點雲轉到 world frame，插入 ikd-Tree
    size_t n_points = cloud_body->size();
    KD_TREE<core::PointXYZIRT>::PointVector points_world(n_points);

// 2. 使用索引 i 進行並行處理
#pragma omp parallel for num_threads(4) schedule(dynamic)
    for (size_t i = 0; i < n_points; ++i) {
        const auto& p = (*cloud_body)[i];
        Eigen::Vector3d pw = T_map_body * Eigen::Vector3d(p.x, p.y, p.z);

        // 透過索引寫入，互不干擾
        points_world[i] = p;  // 複製其餘屬性
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
#pragma omp parallel for num_threads(4) schedule(dynamic)
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

    // 2. 判斷是否為 keyframe（僅供後端 pose graph 使用，與 ikd-Tree 插入無關）
    if (!isNewKeyframe(T_map_body)) return std::nullopt;

    // 3. 建立 KeyFrame（cloud 保存 body frame，不轉換）

    {
        std::unique_lock<std::shared_mutex> lock(keyframe_mutex_);
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
}

void IkdTreeMapBuilder::addKeyFrame(const core::KeyFrame::SharedPtr& keyframe) {
    if (!keyframe || !keyframe->cloud || keyframe->cloud->empty()) return;

    KD_TREE<core::PointXYZIRT>::PointVector points_world;
    points_world.reserve(keyframe->cloud->size());
    for (const auto& p : *keyframe->cloud) {
        Eigen::Vector3d pw = keyframe->pose * Eigen::Vector3d(p.x, p.y, p.z);
        core::PointXYZIRT pt = p;
        pt.x = static_cast<float>(pw.x());
        pt.y = static_cast<float>(pw.y());
        pt.z = static_cast<float>(pw.z());
        points_world.push_back(pt);
    }
    ikd_tree_->Add_Points(points_world, true);  // downsample_on = true

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

// ── queryNearestPoints ────────────────────────────────────────────────────────

std::vector<core::NearestPointResult> IkdTreeMapBuilder::queryNearestPoints(
    const core::PointCloudIRTPtr& query_cloud, const Eigen::Isometry3d& T_map_lidar, int k) const {
    std::vector<core::NearestPointResult> results;
    if (!query_cloud || query_cloud->empty()) return results;
    results.reserve(query_cloud->size());

    auto local_tree = ikd_tree_;

    if (!local_tree) return results;

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
        const_cast<KD_TREE<core::PointXYZIRT>*>(local_tree.get())
            ->Nearest_Search(query_pt, k, neighbors, distances, params_.max_search_dist);

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
    {
        std::lock_guard<std::mutex> lock(increment_mutex_);
        ikd_increment_.clear();
    }

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
    is_updating_map_.store(true);

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
        for (const auto& kf : keyframes_) total_points += kf->cloud->size();
        all_points.resize(total_points);

        size_t current_pos = 0;
        for (const auto& kf : keyframes_) {
            const auto& cloud = kf->cloud;
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
            catch_up_points.swap(ikd_increment_);  // 快速交換，清空桶子
        }
        if (!catch_up_points.empty()) {
            temp_ikd_tree_->Add_Points(catch_up_points, true);  // 這裡耗時，但不卡前端
        }
    }
}

void IkdTreeMapBuilder::updateMap() {
    {
        std::lock_guard<std::mutex> lock(increment_mutex_);
        if (!ikd_increment_.empty()) {
            temp_ikd_tree_->Add_Points(ikd_increment_, true);
            ikd_increment_.clear();
        }
    }

    std::swap(ikd_tree_, temp_ikd_tree_);

    is_updating_map_.store(false);
    last_pending_correction_.reset();
    temp_ikd_tree_.reset();
}

// ── Keyframe 查詢 ─────────────────────────────────────────────────────────────

std::vector<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getAllKeyframes() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    return keyframes_;
}

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getKeyframe(uint64_t id) const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    auto it = keyframe_index_.find(id);
    if (it == keyframe_index_.end()) return std::nullopt;
    return keyframes_[it->second];
}

std::optional<core::KeyFrame::SharedPtr> IkdTreeMapBuilder::getLatestKeyframe() const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
    if (keyframes_.empty()) return std::nullopt;
    return keyframes_.back();
}

core::PointCloudIRTPtr IkdTreeMapBuilder::getGlobalMap() const {
    auto local_tree = ikd_tree_;
    if (!local_tree || local_tree->Root_Node == nullptr)
        return std::make_shared<core::PointCloudIRT>();

    KD_TREE<core::PointXYZIRT>::PointVector all_points;

    // 2. 使用 const_cast 轉為原始指標並呼叫 flatten
    // 注意：ikd_tree_ 是 shared_ptr，我們用 local_tree.get() 拿到指標
    const_cast<KD_TREE<core::PointXYZIRT>*>(local_tree.get())
        ->flatten(local_tree->Root_Node, all_points, NOT_RECORD);

    auto cloud = std::make_shared<core::PointCloudIRT>();
    cloud->reserve(all_points.size());
    for (const auto& p : all_points) cloud->push_back(p);
    return cloud;
}

// ── 私有工具函式 ──────────────────────────────────────────────────────────────

bool IkdTreeMapBuilder::isNewKeyframe(const Eigen::Isometry3d& pose) const {
    std::shared_lock<std::shared_mutex> lock(keyframe_mutex_);
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
