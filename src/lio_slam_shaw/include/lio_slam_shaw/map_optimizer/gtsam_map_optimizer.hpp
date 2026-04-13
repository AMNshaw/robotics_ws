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
    std::vector<double> prior_noise = {1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12};

    std::vector<double> odom_noise = {1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4};

    std::vector<double> odom_noise_degenerate = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

    double isam2_relinearize_threshold = 0.1;
    int isam2_relinearize_skip = 1;
};

class GtsamMapOptimizer : public core::IMapOptimizer {
public:
    explicit GtsamMapOptimizer(const GtsamMapOptimizerParams& params);
    ~GtsamMapOptimizer() override = default;

    void addKeyframe(uint64_t keyframe_id, const core::ScanMatchResult& result) override;

    void addLoopConstraint(const core::LoopConstraint& constraint) override;

    std::vector<std::pair<uint64_t, Eigen::Isometry3d>> optimize() override;

    core::NavState getKeyframePose(uint64_t keyframe_id) const override;

private:
    static gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d& pose);
    static Eigen::Isometry3d fromGtsamPose(const gtsam::Pose3& pose);

    GtsamMapOptimizerParams params_;

    std::unique_ptr<gtsam::ISAM2> optimizer_;

    gtsam::NonlinearFactorGraph loop_graph_;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odom_noise_degenerate_;

    std::map<uint64_t, gtsam::Key> id_to_key_;

    std::map<uint64_t, Eigen::Isometry3d> id_to_pose_;

    bool has_first_keyframe_ = false;
    uint64_t last_keyframe_id_ = 0;

    mutable std::mutex mtx_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__MAP_OPTIMIZER__GTSAM_MAP_OPTIMIZER_HPP_
