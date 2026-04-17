#include "lio_slam_shaw/imu_preintegrator/gtsam_imu_preintegrator.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>

#include <algorithm>

namespace lio_slam_shaw {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

GtsamImuPreintegrator::GtsamImuPreintegrator(const GtsamImuPreintegratorParams& params)
    : params_(params) {
    gtsam_preint_params_ = gtsam::PreintegrationParams::MakeSharedU(params_.gravity);
    gtsam_preint_params_->accelerometerCovariance =
        gtsam::I_3x3 * std::pow(params_.imu_acc_noise, 2);
    gtsam_preint_params_->gyroscopeCovariance = gtsam::I_3x3 * std::pow(params_.imu_gyr_noise, 2);
    gtsam_preint_params_->integrationCovariance = gtsam::I_3x3 * std::pow(1e-4, 2);
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());

    noise_model_between_bias_ =
        (gtsam::Vector(6) << params_.imu_acc_bias_noise, params_.imu_acc_bias_noise,
         params_.imu_acc_bias_noise, params_.imu_gyr_bias_noise, params_.imu_gyr_bias_noise,
         params_.imu_gyr_bias_noise)
            .finished();

    correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    correction_noise_large_ =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());

    T_base_imu_ = toGtsamPose(params_.T_base_imu);
    T_imu_base_ = T_base_imu_.inverse();

    imu_integrator_opt_ =
        std::make_unique<gtsam::PreintegratedImuMeasurements>(gtsam_preint_params_, prior_imu_bias);
    imu_integrator_predict_ =
        std::make_unique<gtsam::PreintegratedImuMeasurements>(gtsam_preint_params_, prior_imu_bias);

    state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
}

void GtsamImuPreintegrator::setImuExtrinsics(const Eigen::Isometry3d& T_base_imu) {
    T_base_imu_ = toGtsamPose(T_base_imu);
    T_imu_base_ = T_base_imu_.inverse();
}

void GtsamImuPreintegrator::integrateImusAndPredict(const std::vector<core::ImuData>& imus) {
    std::lock_guard<std::mutex> lock(mtx_);
    integrateImusAndPredictNoLock(imus);
}

void GtsamImuPreintegrator::integrateImusAndPredictNoLock(const std::vector<core::ImuData>& imus) {
    if (imus.empty()) return;

    // Buffer raw IMU data while waiting for the FIRST LiDAR frame (initial
    // stationary period only).  Once the first frame triggers the init
    // pipeline, we mark the buffer as done and never add more — otherwise
    // post-reset WAITING periods would pollute the buffer with driving data.
    if (state_ == PreintegratorState::WAITING_FOR_FIRST_FRAME) {
        if (!static_imu_done_) {
            for (const auto& imu : imus) {
                if (static_imu_buf_.size() < kStaticImuMaxSamples) {
                    static_imu_buf_.push_back(imu);
                }
            }
        }
        return;
    }

    if (state_ != PreintegratorState::OPTIMIZING) return;

    bool integrated = false;
    for (size_t i = 0; i < imus.size(); ++i) {
        const auto& imu = imus[i];
        if (imu.timestamp < last_imu_.timestamp) continue;

        double dt = 1.0 / 500.0;
        Eigen::Vector3d mean_acc = imu.acc;
        Eigen::Vector3d mean_gyr = imu.gyr;

        if (last_imu_.timestamp.time_since_epoch().count() != 0) {
            std::chrono::duration<double> diff = imu.timestamp - last_imu_.timestamp;

            dt = diff.count();
            if (dt <= 0.0 || dt > 0.1) dt = 1.0 / 500.0;

            mean_acc = (last_imu_.acc + imu.acc) / 2.0;
            mean_gyr = (last_imu_.gyr + imu.gyr) / 2.0;
        }

        imu_integrator_predict_->integrateMeasurement(mean_acc, mean_gyr, dt);
        integrated = true;

        const auto gtsam_mid =
            imu_integrator_predict_->predict(last_optimized_state_, last_optimized_bias_);

        const Eigen::Vector3d gyr_corrected =
            mean_gyr - Eigen::Vector3d(last_optimized_bias_.gyroscope().x(),
                                       last_optimized_bias_.gyroscope().y(),
                                       last_optimized_bias_.gyroscope().z());
        auto nav_state = fromGtsamNavState(imu.timestamp, gtsam_mid);
        nav_state.pose = nav_state.pose * Eigen::Isometry3d(T_imu_base_.matrix());
        nav_state.angular_vel =
            Eigen::Matrix3d(T_base_imu_.rotation().matrix().cast<double>()) * gyr_corrected;
        nav_state_queue_.push_back(nav_state);
        if (nav_state_queue_.size() > 2000) {
            nav_state_queue_.pop_front();
        }

        last_imu_ = imu;
    }
    if (!integrated) return;

    curr_state_ = nav_state_queue_.back();
}

void GtsamImuPreintegrator::updateBiasAndRepropagateImus(
    const core::ScanMatchResult& scan_match_result,
    const std::vector<core::ImuData>& opt_imu_segment,
    const std::vector<core::ImuData>& reprop_imu_segment) {
    std::lock_guard<std::mutex> lock(mtx_);

    const gtsam::Pose3 current_pose = toGtsamPose(scan_match_result.pose);
    const gtsam::Pose3 imu_pose = current_pose.compose(T_base_imu_);

    if (state_ == PreintegratorState::WAITING_FOR_FIRST_FRAME) {
        // Gate: discard LiDAR frames until we have enough static IMU data
        // for a reliable bias estimate.  At 500Hz this is ~0.4s.
        if (static_imu_buf_.size() < kMinStaticSamples) {
            std::cerr << "[ImuPreintegrator] Waiting for static IMU data: "
                      << static_imu_buf_.size() << "/" << kMinStaticSamples << std::endl;
            return;
        }
        // Compute bias directly from static IMU data and go straight to ISAM2.
        // Multi-frame batch optimization was removed — the static mean is reliable
        // enough, and the 10-frame delay costs valuable time.
        static_imu_done_ = true;
        gtsam::imuBias::ConstantBias init_bias = solveInitBias();
        std::cerr << "[ImuPreintegrator] Static init done. bias_acc="
                  << init_bias.accelerometer().transpose()
                  << " bias_gyr=" << init_bias.gyroscope().transpose() << std::endl;

        resetOptimization();
        initFirstFrame(imu_pose, init_bias);

        imu_integrator_predict_->resetIntegrationAndSetBias(last_optimized_bias_);
        state_ = PreintegratorState::OPTIMIZING;
        last_scan_pose_imu_ = imu_pose;

        nav_state_queue_.clear();
        integrateImusAndPredictNoLock(reprop_imu_segment);
        return;
    }

    if (state_ == PreintegratorState::INITIALIZING) {
        // Legacy path — should not be reached since we skip INITIALIZING now.
        // Treat as WAITING and reset.
        state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
        return;
    }

    try {
        if (graph_node_index_ == params_.marginalization_threshold) {
            marginalizeOldFactors();
            graph_node_index_ = 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ImuPreintegrator] Marginalization failed, resetting: " << e.what()
                  << std::endl;
        state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
        return;
    }

    // Use fixed correction noise (LIO-SAM style).
    // Scan matcher H^{-1} covariance is unreliable — it underestimates uncertainty
    // in degenerate / ambiguous geometry, causing ISAM2 to over-trust bad matches.
    gtsam::SharedNoiseModel pose_noise =
        scan_match_result.is_degenerate ? correction_noise_large_ : correction_noise_;

    if (!calculateImuBias(imu_pose, pose_noise, opt_imu_segment)) {
        state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
        return;
    }

    imu_integrator_predict_->resetIntegrationAndSetBias(last_optimized_bias_);
    state_ = PreintegratorState::OPTIMIZING;

    nav_state_queue_.clear();

    integrateImusAndPredictNoLock(reprop_imu_segment);

    graph_node_index_++;
}

core::NavState GtsamImuPreintegrator::getLatestPredictState() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return curr_state_;
}

std::vector<core::NavState> GtsamImuPreintegrator::getNavStateQueueSnapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return std::vector<core::NavState>(nav_state_queue_.begin(), nav_state_queue_.end());
}

std::optional<core::NavState> GtsamImuPreintegrator::queryNavState(const core::Timestamp& t) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (nav_state_queue_.empty()) return std::nullopt;

    if (t <= nav_state_queue_.front().timestamp) return nav_state_queue_.front();
    if (t >= nav_state_queue_.back().timestamp) return nav_state_queue_.back();

    auto it = std::lower_bound(
        nav_state_queue_.begin(), nav_state_queue_.end(), t,
        [](const core::NavState& s, const core::Timestamp& ts) { return s.timestamp < ts; });

    const auto& s1 = *it;
    const auto& s0 = *(it - 1);

    double total = core::getDeltaSec(s0.timestamp, s1.timestamp);
    if (total < 1e-9) return s0;

    double alpha = std::clamp(core::getDeltaSec(s0.timestamp, t) / total, 0.0, 1.0);

    Eigen::Quaterniond q0(s0.pose.linear());
    Eigen::Quaterniond q1(s1.pose.linear());

    core::NavState result;
    result.timestamp = t;
    result.pose.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
    result.pose.translation() =
        s0.pose.translation() + alpha * (s1.pose.translation() - s0.pose.translation());
    result.linear_vel = s0.linear_vel + alpha * (s1.linear_vel - s0.linear_vel);
    result.angular_vel = s0.angular_vel + alpha * (s1.angular_vel - s0.angular_vel);
    result.acc_bias = s0.acc_bias;
    result.gyr_bias = s0.gyr_bias;
    result.pose_cov = s0.pose_cov;
    return result;
}

void GtsamImuPreintegrator::initFirstFrame(const gtsam::Pose3& imu_pose,
                                           const gtsam::imuBias::ConstantBias& init_bias) {
    // Bias prior: very tight σ=0.01 freezes bias near the static-mean estimate.
    // The chain of BetweenFactors preserves relative smoothness, but a loose
    // prior lets the ImuFactor+PosePrior accumulate enough pull to drag bias
    // toward zero over ~20 frames. σ=0.01 keeps bias anchored.
    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-2);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), imu_pose, correction_noise_));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0), prior_vel_noise));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), init_bias, prior_bias_noise));
    values.insert(X(0), imu_pose);
    values.insert(V(0), gtsam::Vector3(0, 0, 0));
    values.insert(B(0), init_bias);
    optimizer_->update(graph, values);

    last_optimized_state_ = gtsam::NavState(imu_pose, gtsam::Vector3(0, 0, 0));
    last_optimized_bias_ = init_bias;

    imu_integrator_opt_->resetIntegrationAndSetBias(last_optimized_bias_);
    imu_integrator_predict_->resetIntegrationAndSetBias(last_optimized_bias_);

    graph_node_index_ = 1;
}

void GtsamImuPreintegrator::marginalizeOldFactors() {
    gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_->marginalCovariance(X(graph_node_index_ - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_->marginalCovariance(V(graph_node_index_ - 1)));
    gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise =
        gtsam::noiseModel::Gaussian::Covariance(
            optimizer_->marginalCovariance(B(graph_node_index_ - 1)));
    const auto& init_pose = last_optimized_state_.pose();
    const auto& init_vel = last_optimized_state_.velocity();
    const auto& init_bias = last_optimized_bias_;

    resetOptimization();
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), init_pose, updatedPoseNoise));
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(0), init_vel, updatedVelNoise));
    graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), init_bias, updatedBiasNoise));
    values.insert(X(0), init_pose);
    values.insert(V(0), init_vel);
    values.insert(B(0), init_bias);
    optimizer_->update(graph, values);
}

gtsam::imuBias::ConstantBias GtsamImuPreintegrator::solveInitBias() {
    // Compute initial IMU bias from static data collected during the stationary
    // period before the first LiDAR frame.
    //
    // Gyro bias: direct mean of gyro measurements (true rotation ≈ 0 when stationary).
    // Acc bias: acc_mean − (0, 0, g).  Assumes the IMU z-axis is roughly upward
    //           (valid for ground vehicles on flat terrain).

    Eigen::Vector3d gyr_mean = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_mean = Eigen::Vector3d::Zero();

    if (!static_imu_buf_.empty()) {
        for (const auto& imu : static_imu_buf_) {
            gyr_mean += imu.gyr;
            acc_mean += imu.acc;
        }
        const double n = static_cast<double>(static_imu_buf_.size());
        gyr_mean /= n;
        acc_mean /= n;
    }

    // acc_bias = acc_meas_mean − (0, 0, g)
    Eigen::Vector3d acc_bias = acc_mean - Eigen::Vector3d(0.0, 0.0, params_.gravity);

    std::cerr << "[ImuPreintegrator] solveInitBias: static " << static_imu_buf_.size()
              << " samples, acc_mean=" << acc_mean.transpose()
              << ", gyr_mean=" << gyr_mean.transpose() << " → acc_bias=" << acc_bias.transpose()
              << ", gyr_bias=" << gyr_mean.transpose() << std::endl;

    return gtsam::imuBias::ConstantBias(acc_bias, gyr_mean);
}

bool GtsamImuPreintegrator::calculateImuBias(const gtsam::Pose3& imu_pose,
                                             gtsam::SharedNoiseModel pose_cov,
                                             const std::vector<core::ImuData>& opt_imu_segment) {
    for (size_t i = 1; i < opt_imu_segment.size(); ++i) {
        const auto& curr_imu = opt_imu_segment[i];
        const auto& prev_imu = opt_imu_segment[i - 1];
        std::chrono::duration<double> diff = curr_imu.timestamp - prev_imu.timestamp;
        double dt = diff.count();
        if (dt <= 0.0 || dt > 0.1) {
            dt = 1.0 / 500.0;
        }

        Eigen::Vector3d avg_acc = (curr_imu.acc + prev_imu.acc) / 2.0;
        Eigen::Vector3d avg_gyr = (curr_imu.gyr + prev_imu.gyr) / 2.0;

        imu_integrator_opt_->integrateMeasurement(avg_acc, avg_gyr, dt);
    }

    const double dt_between = std::max(imu_integrator_opt_->deltaTij(), 1e-5);

    std::cerr << "[ImuPreintegrator] calculateImuBias: imu_seg=" << opt_imu_segment.size()
              << " deltaTij=" << dt_between << " node=" << graph_node_index_
              << " vel=" << last_optimized_state_.velocity().transpose()
              << " abias=" << last_optimized_bias_.accelerometer().transpose()
              << " gbias=" << last_optimized_bias_.gyroscope().transpose() << std::endl;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;

    const gtsam::PreintegratedImuMeasurements& preint_imu = *imu_integrator_opt_;
    graph.add(gtsam::ImuFactor(X(graph_node_index_ - 1), V(graph_node_index_ - 1),
                               X(graph_node_index_), V(graph_node_index_), B(graph_node_index_ - 1),
                               preint_imu));
    graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(graph_node_index_ - 1), B(graph_node_index_), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(sqrt(dt_between) * noise_model_between_bias_)));

    // Use PriorFactor with fixed noise (LIO-SAM style).
    // PriorFactor anchors absolute pose, preventing velocity drift.
    // BetweenFactor only constrains relative motion and allows unbounded drift.
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(graph_node_index_), imu_pose, pose_cov));

    // --- Scan-matcher-derived velocity prior ---
    // In FAST-LIO2, point-to-plane residuals directly update velocity in the EKF.
    // In our loose coupling, velocity is only indirectly inferred from ImuFactor +
    // PriorFactor(pose). A small attitude error causes gravity → velocity divergence.
    // Bridging the gap: estimate velocity from consecutive scan-matched poses
    // and add as a soft prior. This directly constrains velocity from LiDAR info.
    if (last_scan_pose_imu_.has_value()) {
        Eigen::Vector3d dp = imu_pose.translation() - last_scan_pose_imu_->translation();
        Eigen::Vector3d v_scan = dp / dt_between;
        gtsam::noiseModel::Diagonal::shared_ptr vel_noise =
            gtsam::noiseModel::Isotropic::Sigma(3, 1.0);  // σ=1 m/s
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(V(graph_node_index_), v_scan, vel_noise));
    }
    last_scan_pose_imu_ = imu_pose;

    gtsam::NavState prop_state =
        imu_integrator_opt_->predict(last_optimized_state_, last_optimized_bias_);
    values.insert(X(graph_node_index_), prop_state.pose());
    values.insert(V(graph_node_index_), prop_state.velocity());
    values.insert(B(graph_node_index_), last_optimized_bias_);

    try {
        optimizer_->update(graph, values);
        optimizer_->update();

        gtsam::Values result = optimizer_->calculateEstimate();
        last_optimized_state_ = gtsam::NavState(result.at<gtsam::Pose3>(X(graph_node_index_)),
                                                result.at<gtsam::Vector3>(V(graph_node_index_)));
        last_optimized_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(graph_node_index_));
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
        std::cerr << "[ImuPreintegrator] ISAM2 indeterminant system, resetting: " << e.what()
                  << std::endl;
        resetOptimization();
        last_scan_pose_imu_.reset();
        return false;
    }

    imu_integrator_opt_->resetIntegrationAndSetBias(last_optimized_bias_);

    return !failureDetection(last_optimized_state_.velocity(), last_optimized_bias_);
}

bool GtsamImuPreintegrator::failureDetection(const gtsam::Vector3& curr_vel,
                                             const gtsam::imuBias::ConstantBias& curr_bias) {
    Eigen::Vector3f vel(curr_vel.x(), curr_vel.y(), curr_vel.z());
    if (vel.norm() > 30) {
        return true;
    }

    Eigen::Vector3f ba(curr_bias.accelerometer().x(), curr_bias.accelerometer().y(),
                       curr_bias.accelerometer().z());
    Eigen::Vector3f bg(curr_bias.gyroscope().x(), curr_bias.gyroscope().y(),
                       curr_bias.gyroscope().z());
    if (ba.norm() > 15.0 || bg.norm() > 1.0) {
        return true;
    }

    return false;
}

void GtsamImuPreintegrator::resetOptimization() {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optParameters.factorization =
        gtsam::ISAM2Params::QR;  // QR is more numerically stable than Cholesky
    optimizer_ = std::make_unique<gtsam::ISAM2>(optParameters);
}

gtsam::Pose3 GtsamImuPreintegrator::toGtsamPose(const Eigen::Isometry3d& pose) const {
    return gtsam::Pose3(pose.matrix());
}
core::NavState GtsamImuPreintegrator::fromGtsamNavState(const core::Timestamp& timestamp,
                                                        const gtsam::NavState& g_state) const {
    core::NavState state;
    state.timestamp = timestamp;
    state.pose = Eigen::Isometry3d(g_state.pose().matrix());
    state.linear_vel = g_state.velocity();
    state.acc_bias = Eigen::Vector3d(last_optimized_bias_.accelerometer().x(),
                                     last_optimized_bias_.accelerometer().y(),
                                     last_optimized_bias_.accelerometer().z());
    state.gyr_bias =
        Eigen::Vector3d(last_optimized_bias_.gyroscope().x(), last_optimized_bias_.gyroscope().y(),
                        last_optimized_bias_.gyroscope().z());
    return state;
}

}  // namespace lio_slam_shaw