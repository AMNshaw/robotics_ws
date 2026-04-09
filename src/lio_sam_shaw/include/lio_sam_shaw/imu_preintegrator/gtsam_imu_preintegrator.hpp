#ifndef LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_
#define LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_

#include <chrono>
#include <deque>
#include <memory>
#include <mutex>

// Core 介面
#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"

// GTSAM 依賴
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace lio_slam_shaw {

// 透過 Factory 或 Node 傳入的參數
struct GtsamImuPreintegratorParams {
    double gravity = 9.80511;
    double imu_acc_noise = 3.9939570888238808e-03;
    double imu_gyr_noise = 1.5636343949698187e-03;
    double imu_acc_bias_noise = 6.4356659353532566e-05;
    double imu_gyr_bias_noise = 3.5640318696367613e-05;

    size_t marginalization_threshold_ = 100;

    std::vector<double> T_base_imu_trans = {0.0, 0.0, 0.0};
    std::vector<double> T_base_imu_rot = {1.0, 0.0, 0.0, 0.0};
};

enum class PreintegratorState { WAITING_FOR_FIRST_FRAME, INITIALIZED, OPTIMIZING };

class GtsamImuPreintegrator : public core::IImuPreintegrator {
public:
    explicit GtsamImuPreintegrator(const GtsamImuPreintegratorParams& params);
    ~GtsamImuPreintegrator() override = default;

    void integrateImusAndPredict(const std::vector<core::ImuData>& imus) override;
    void updateBiasAndRepropagateImus(
        const core::NavState& optimized_state, const std::vector<core::ImuData>& opt_imu_segment,
        const std::vector<core::ImuData>& reprop_imu_segment) override;
    core::NavState getLatestNavState() const override;
    std::optional<core::NavState> queryNavState(const core::Timestamp& t) const override;

private:
    // ---- functions----
    void integrateImusAndPredictNoLock(const std::vector<core::ImuData>& imus);
    void initFirstFrame(const gtsam::Pose3& current_pose);
    void resetOptimization();
    void marginalizeOldFactors();
    bool calculateImuBias(const gtsam::Pose3& current_pose,
                          const Eigen::Matrix<double, 6, 6>& pose_cov,
                          const std::vector<core::ImuData>& opt_imu_segment);
    bool failureDetection(const gtsam::Vector3& velCur,
                          const gtsam::imuBias::ConstantBias& biasCur);

    gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d& pose) const;
    core::NavState fromGtsamNavState(const gtsam::NavState& g_state,
                                     const core::Timestamp& timestamp) const;

    // ---- members----

    GtsamImuPreintegratorParams params_;
    gtsam::Pose3 T_base_imu_;
    gtsam::Pose3 T_imu_base_;
    PreintegratorState state_;
    mutable std::mutex mtx_;

    std::unique_ptr<gtsam::ISAM2> optimizer_;
    gtsam::NonlinearFactorGraph factor_graph_;
    gtsam::Values initial_graph_values_;
    uint64_t graph_node_index_ = 0;

    // GTSAM 參數
    boost::shared_ptr<gtsam::PreintegrationParams> gtsam_preint_params_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_large_;
    gtsam::Vector noise_model_between_bias_;

    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_opt_;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_predict_;

    gtsam::NavState last_optimized_state_;
    gtsam::imuBias::ConstantBias last_optimized_bias_;

    core::NavState curr_state_{};
    core::ImuData last_imu_{};

    // 每次 integrateImusAndPredict 的中間預測狀態，供 deskew 插值查詢
    // 在 updateBiasAndRepropagateImus 重新設定 bias 前清空，再由 repropagation 重建
    std::deque<core::NavState> nav_state_queue_;
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_