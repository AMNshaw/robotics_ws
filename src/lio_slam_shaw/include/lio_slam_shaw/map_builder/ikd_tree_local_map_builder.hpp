#ifndef LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_LOCAL_MAP_BUILDER_HPP_
#define LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_LOCAL_MAP_BUILDER_HPP_

#include <Eigen/Geometry>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <vector>

#include "lio_slam_shaw/core/i_local_map_builder.hpp"
#include "lio_slam_shaw/core/keyframe.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree.h"

namespace lio_slam_shaw::map_builder {

class IkdTreeLocalReadSession;  // forward decl

struct IkdTreeLocalMapBuilderParams {
    float ikd_delete_param = 0.5f;
    float ikd_balance_param = 0.6f;
    float ikd_downsample_size = 0.3f;
    double box_trim_half_length = 50.0;
    Eigen::Isometry3d T_base_lidar = Eigen::Isometry3d::Identity();
};

/// Local map builder backed by an incrementally-updated KD-tree.
/// Never rebuilt — only grows via addScan and shrinks via boxTrim.
class IkdTreeLocalMapBuilder : public core::ILocalMapBuilder {
public:
    using SharedPtr = std::shared_ptr<IkdTreeLocalMapBuilder>;

    explicit IkdTreeLocalMapBuilder(const IkdTreeLocalMapBuilderParams& params = {});
    ~IkdTreeLocalMapBuilder() override = default;

    void addScan(const core::LidarFrame::SharedPtr& frame) override;
    bool isMapReady() const override;

    /// Reset the tree (used by initializer / internal components that own their own local map).
    void clearMap();

    /// Direct addKeyFrame for components (initializer, loop closure detector) that
    /// build a local map from keyframes.
    void addKeyFrame(const core::Keyframe::SharedPtr& keyframe);

private:
    friend class IkdTreeLocalReadSession;

    void boxTrim(const Eigen::Vector3d& center, double half_length);

    IkdTreeLocalMapBuilderParams params_;
    bool is_first_frame_ = true;
    mutable std::shared_mutex ikd_tree_mutex_;
    std::shared_ptr<KD_TREE<core::PointXYZIRT>> ikd_tree_;
};

// ─────────────────────────────────────────────────────────────────────────────
/// RAII read-session: holds shared_lock on the ikd-tree for batched KNN.
class IkdTreeLocalReadSession {
public:
    using PointVector = KD_TREE<core::PointXYZIRT>::PointVector;

    explicit IkdTreeLocalReadSession(const IkdTreeLocalMapBuilder& owner);

    IkdTreeLocalReadSession(IkdTreeLocalReadSession&&) noexcept = default;
    IkdTreeLocalReadSession& operator=(IkdTreeLocalReadSession&&) noexcept = default;
    IkdTreeLocalReadSession(const IkdTreeLocalReadSession&) = delete;
    IkdTreeLocalReadSession& operator=(const IkdTreeLocalReadSession&) = delete;

    bool searchKNearest(const core::PointXYZIRT& query_pt, int k, float search_dist,
                        const PointVector*& out_neighbors,
                        const std::vector<float>*& out_distances) const;

private:
    const IkdTreeLocalMapBuilder* owner_;
    std::shared_lock<std::shared_mutex> lock_;
};

}  // namespace lio_slam_shaw::map_builder

#endif  // LIO_SLAM_SHAW__MAP_BUILDER__IKD_TREE_LOCAL_MAP_BUILDER_HPP_
