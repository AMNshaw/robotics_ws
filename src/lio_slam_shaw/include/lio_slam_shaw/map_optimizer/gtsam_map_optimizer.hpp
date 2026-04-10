#ifndef LIO_SLAM_SHAW__MAP_OPTIMIZER__GTSAM_MAP_OPTIMIZER_HPP_
#define LIO_SLAM_SHAW__MAP_OPTIMIZER__GTSAM_MAP_OPTIMIZER_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "lio_slam_shaw/core/i_map_optimizer.hpp"

namespace lio_slam_shaw {

struct GtsamMapOptimizerParams {
    // 6-DoF diagonal sigmas: [roll, pitch, yaw, x, y, z]

    // 第一個 keyframe 的 prior noise（幾乎 fix 住）
    std::vector<double> prior_noise = {1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12};

    // 正常情況下 odometry BetweenFactor 的 noise
    std::vector<double> odom_noise = {1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4};

    // 退化環境（is_degenerate=true）時放大 odometry noise
    std::vector<double> odom_noise_degenerate = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    double isam2_relinearize_threshold = 0.1;
    int isam2_relinearize_skip = 1;
};

// GTSAM iSAM2-based 全局位姿圖優化器
// addKeyframe 每幀都做一次增量更新，以維持最新估計
// optimize 在 loop closure 後額外執行多輪更新，回傳所有 keyframe 的修正後 pose
class GtsamMapOptimizer : public core::IMapOptimizer {
public:
    explicit GtsamMapOptimizer(const GtsamMapOptimizerParams& params);
    ~GtsamMapOptimizer() override = default;

    // 加入新 keyframe 並做一次增量 iSAM2 更新
    void addKeyframe(uint64_t keyframe_id, const core::ScanMatchResult& result) override;

    // 將 loop closure BetweenFactor 加入下一次 optimize() 的批次中
    void addLoopConstraint(const core::LoopConstraint& constraint) override;

    // 執行多輪 iSAM2 更新（loop closure 收斂），回傳所有 keyframe 修正後 pose
    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> optimize() override;

    // 回傳指定 keyframe 目前最佳估計
    core::NavState getKeyframePose(uint64_t keyframe_id) const override;

private:
    static gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d& pose);
    static Eigen::Isometry3d fromGtsamPose(const gtsam::Pose3& pose);

    GtsamMapOptimizerParams params_;

    std::unique_ptr<gtsam::ISAM2> optimizer_;

    // loop closure factor 批次（在 optimize() 時一起送進去）
    gtsam::NonlinearFactorGraph loop_graph_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_degenerate_;

    // keyframe_id → gtsam::Key（直接用 id，不額外 re-map）
    // 用 ordered map 方便按序遍歷
    std::map<uint64_t, gtsam::Key> id_to_key_;

    // 最新估計：keyframe_id → T_w_body
    std::map<uint64_t, Eigen::Isometry3d> id_to_pose_;

    bool has_first_keyframe_ = false;
    uint64_t last_keyframe_id_ = 0;

    mutable std::mutex mtx_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__MAP_OPTIMIZER__GTSAM_MAP_OPTIMIZER_HPP_
