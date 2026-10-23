#ifndef LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_
#define LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/shared_ptr.hpp>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "lio_slam_shaw/core/i_imu_preintegrator.hpp"

namespace lio_slam_shaw {

struct GtsamImuPreintegratorParams {
    double gravity = 9.80511;
    double imu_acc_noise = 3.9939570888238808e-03;
    double imu_gyr_noise = 1.5636343949698187e-03;
    double imu_acc_bias_noise = 6.4356659353532566e-05;
    double imu_gyr_bias_noise = 3.5640318696367613e-05;

    size_t marginalization_threshold = 100;
    size_t init_frames = 10;  // number of LiDAR frames for multi-frame initialization

    Eigen::Isometry3d T_base_imu = Eigen::Isometry3d::Identity();
};

enum class PreintegratorState { WAITING_FOR_FIRST_FRAME, INITIALIZING, OPTIMIZING };

class GtsamImuPreintegrator : public core::IImuPreintegrator {
public:
    explicit GtsamImuPreintegrator(const GtsamImuPreintegratorParams& params);
    ~GtsamImuPreintegrator() override = default;

    void setImuExtrinsics(const Eigen::Isometry3d& T_base_imu) override;
    void integrateImusAndPredict(const std::vector<core::ImuData>& imus) override;
    void updateBiasAndRepropagateImus(
        const core::ScanMatchResult& scan_match_result,
        const std::vector<core::ImuData>& opt_imu_segment,
        const std::vector<core::ImuData>& reprop_imu_segment) override;
    core::NavState getLatestPredictState() const override;
    std::optional<core::NavState> queryNavState(const core::Timestamp& t) const override;
    std::vector<core::NavState> getNavStateQueueSnapshot() const override;

private:
    void integrateImusAndPredictNoLock(const std::vector<core::ImuData>& imus);
    void initFirstFrame(
        const gtsam::Pose3& current_pose,
        const gtsam::imuBias::ConstantBias& init_bias = gtsam::imuBias::ConstantBias());
    void resetOptimization();
    void marginalizeOldFactors();
    bool calculateImuBias(const gtsam::Pose3& current_pose, gtsam::SharedNoiseModel pose_noise,
                          const std::vector<core::ImuData>& opt_imu_segment);
    bool failureDetection(const gtsam::Vector3& velCur,
                          const gtsam::imuBias::ConstantBias& biasCur);
    /// Multi-frame batch initialization: solve for velocities and IMU bias
    /// using collected scan-matched poses and IMU data.
    gtsam::imuBias::ConstantBias solveInitBias();

    gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d& pose) const;
    core::NavState fromGtsamNavState(const core::Timestamp& timestamp,
                                     const gtsam::NavState& g_state) const;

    GtsamImuPreintegratorParams params_;
    gtsam::Pose3 T_base_imu_;
    gtsam::Pose3 T_imu_base_;
    PreintegratorState state_;
    mutable std::mutex mtx_;

    std::unique_ptr<gtsam::ISAM2> optimizer_;
    uint64_t graph_node_index_ = 0;

    boost::shared_ptr<gtsam::PreintegrationParams> gtsam_preint_params_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_large_;
    gtsam::Vector noise_model_between_bias_;

    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_opt_;
    std::unique_ptr<gtsam::PreintegratedImuMeasurements> imu_integrator_predict_;

    gtsam::NavState last_optimized_state_;
    gtsam::imuBias::ConstantBias last_optimized_bias_;
    std::optional<gtsam::Pose3> last_scan_pose_imu_;  // previous scan matcher pose (IMU frame)

    core::NavState curr_state_{};
    core::ImuData last_imu_{};

    std::deque<core::NavState> nav_state_queue_;

    // Multi-frame initialization buffers
    struct InitFrame {
        gtsam::Pose3 imu_pose;
        std::vector<core::ImuData> imu_segment;  // IMU data between this frame and the next
    };
    std::vector<InitFrame> init_frames_buf_;

    // Raw IMU data collected while waiting for the first LiDAR frame.
    // Used to compute the static gyro mean as a bias seed.
    static constexpr size_t kStaticImuMaxSamples = 500;
    static constexpr size_t kMinStaticSamples = 200;  // gate: need ≥200 samples (~0.4s)
    std::vector<core::ImuData> static_imu_buf_;
    bool static_imu_done_ = false;  // true once the first LiDAR frame arrives — stops buffering
};

}  // namespace lio_slam_shaw

#endif  // LIO_SLAM_SHAW__IMU_PREINTEGRATOR__GTSAM_IMU_PREINTEGRATOR_HPP_