#ifndef LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_

#include <Eigen/Geometry>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_map_builder.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree.h"

namespace lio_slam_shaw::map_builder {

class IkdTreeReadSession;  // defined below, after IkdTreeMapBuilder

struct IkdTreeMapBuilderParams {
    double keyframe_distance_threshold = 1.0;
    double keyframe_angle_threshold = 0.2;

    float ikd_delete_param = 0.5f;
    float ikd_balance_param = 0.6f;
    float ikd_downsample_size = 0.3f;
    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
};

class IkdTreeMapBuilder : public core::IMapBuilder {
public:
    explicit IkdTreeMapBuilder(const IkdTreeMapBuilderParams& params = {});
    ~IkdTreeMapBuilder() override = default;

    std::optional<core::Keyframe::SharedPtr> addFrame(
        const core::LidarFrame::SharedPtr& frame) override;
    void addKeyFrame(const core::Keyframe::SharedPtr& keyframe) override;

    void clearMap() override;

    /// Returns true once the ikd-tree has been built (at least one frame added).
    bool isMapReady() const override {
        std::shared_lock<std::shared_mutex> lock(ikd_tree_mutex_);
        return !is_first_frame_;
    }

    void updateKeyframePoses(
        const std::vector<std::pair<uint64_t, Eigen::Isometry3d>>& id_pose_pairs) override;

    void updateMap() override;

    std::vector<core::Keyframe::SharedPtr> getAllKeyframes() const override;
    std::optional<core::Keyframe::SharedPtr> getKeyframe(uint64_t id) const override;
    std::optional<core::Keyframe::SharedPtr> getLatestKeyframe() const override;
    core::PointCloudIRTPtr getGlobalMap() const override;

private:
    bool isNewKeyframe(const Eigen::Isometry3d& pose) const;

    // IkdTreeReadSession is granted access to the ikd-tree and its mutex so it can take the
    // read lock and run zero-copy KNN queries inside a batched session.
    friend class IkdTreeReadSession;

    IkdTreeMapBuilderParams params_;

    bool is_first_frame_ = true;
    mutable std::shared_mutex ikd_tree_mutex_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> ikd_tree_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> temp_ikd_tree_;

    std::atomic<bool> is_updating_map_{false};
    mutable std::mutex increment_mutex_;
    KD_TREE<core::PointXYZIRT>::PointVector ikd_increment_;
    mutable std::shared_mutex correction_mutex_;
    std::optional<Eigen::Isometry3d> last_pending_correction_;

    mutable std::shared_mutex keyframe_mutex_;
    std::vector<core::Keyframe::SharedPtr> keyframes_;
    std::unordered_map<uint64_t, size_t> keyframe_index_;  // id → keyframes_ 的 index

    std::atomic<uint64_t> next_keyframe_id_{0};
};

// ─────────────────────────────────────────────────────────────────────────────────────────────────
/// RAII read-session over an IkdTreeMapBuilder's internal ikd-tree.  Acquires the map builder's
/// internal read lock for the session's lifetime so a batch of KNN queries (e.g. iEKF Phase-1
/// parallel KNN over thousands of scan points × max_iterations) avoids per-query shared_mutex
/// atomics.  Uses friend access — no public API pollution on the map builder.
class IkdTreeReadSession {
public:
    using PointVector = KD_TREE<core::PointXYZIRT>::PointVector;

    /// Acquire the read lock on `owner`'s ikd-tree.  The session must outlive any
    /// pointer returned from `searchKNearest()`.
    explicit IkdTreeReadSession(const IkdTreeMapBuilder& owner);

    IkdTreeReadSession(IkdTreeReadSession&&) noexcept = default;
    IkdTreeReadSession& operator=(IkdTreeReadSession&&) noexcept = default;
    IkdTreeReadSession(const IkdTreeReadSession&) = delete;
    IkdTreeReadSession& operator=(const IkdTreeReadSession&) = delete;

    /// KNN inside the held read lock.  Output buffers reference thread_local storage and remain
    /// valid until the next call on the same thread.
    bool searchKNearest(const core::PointXYZIRT& query_pt, int k, float search_dist,
                        const PointVector*& out_neighbors,
                        const std::vector<float>*& out_distances) const;

private:
    const IkdTreeMapBuilder* owner_;
    std::shared_lock<std::shared_mutex> lock_;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_MAP_BUILDER_HPP_
