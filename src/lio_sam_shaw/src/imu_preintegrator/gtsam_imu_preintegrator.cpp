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

    const auto& trans = params_.T_base_imu_trans;
    const auto& rot = params_.T_base_imu_rot;
    T_base_imu_ = gtsam::Pose3(gtsam::Rot3(rot[0], rot[1], rot[2], rot[3]),
                               gtsam::Point3(trans[0], trans[1], trans[2]));
    T_imu_base_ = T_base_imu_.inverse();

    imu_integrator_opt_ =
        std::make_unique<gtsam::PreintegratedImuMeasurements>(gtsam_preint_params_, prior_imu_bias);
    imu_integrator_predict_ =
        std::make_unique<gtsam::PreintegratedImuMeasurements>(gtsam_preint_params_, prior_imu_bias);

    state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
}

void GtsamImuPreintegrator::integrateImusAndPredict(const std::vector<core::ImuData>& imus) {
    std::lock_guard<std::mutex> lock(mtx_);
    integrateImusAndPredictNoLock(imus);
}

void GtsamImuPreintegrator::integrateImusAndPredictNoLock(const std::vector<core::ImuData>& imus) {
    if (imus.empty()) return;
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

        // 將當前預測狀態存入 queue，供 deskew 插值使用
        const auto gtsam_mid =
            imu_integrator_predict_->predict(last_optimized_state_, last_optimized_bias_);
        nav_state_queue_.push_back(fromGtsamNavState(gtsam_mid, imu.timestamp));

        last_imu_ = imu;
    }
    if (!integrated) return;

    curr_state_ = nav_state_queue_.back();
}

void GtsamImuPreintegrator::updateBiasAndRepropagateImus(
    const core::NavState& optimized_state, const std::vector<core::ImuData>& opt_imu_segment,
    const std::vector<core::ImuData>& reprop_imu_segment) {
    std::lock_guard<std::mutex> lock(mtx_);

    const gtsam::Pose3 current_pose = toGtsamPose(optimized_state.pose);
    const gtsam::Pose3 imu_pose = current_pose.compose(T_base_imu_);

    if (state_ == PreintegratorState::WAITING_FOR_FIRST_FRAME) {
        resetOptimization();
        initFirstFrame(imu_pose);
        state_ = PreintegratorState::INITIALIZED;
        return;
    }

    if (graph_node_index_ == params_.marginalization_threshold_) {
        marginalizeOldFactors();
        graph_node_index_ = 1;
    }

    if (!calculateImuBias(imu_pose, optimized_state.pose_cov, opt_imu_segment)) {
        state_ = PreintegratorState::WAITING_FOR_FIRST_FRAME;
        return;
    }

    imu_integrator_predict_->resetIntegrationAndSetBias(last_optimized_bias_);
    state_ = PreintegratorState::OPTIMIZING;

    // bias 已更新，舊的預測狀態全部作廢；repropagation 會重新填入新狀態
    nav_state_queue_.clear();

    integrateImusAndPredictNoLock(reprop_imu_segment);

    graph_node_index_++;
}

core::NavState GtsamImuPreintegrator::getLatestNavState() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return curr_state_;
}

std::optional<core::NavState> GtsamImuPreintegrator::queryNavState(const core::Timestamp& t) const {
    std::lock_guard<std::mutex> lock(mtx_);

    if (nav_state_queue_.empty()) return std::nullopt;

    // clamp 到 queue 兩端
    if (t <= nav_state_queue_.front().timestamp) return nav_state_queue_.front();
    if (t >= nav_state_queue_.back().timestamp) return nav_state_queue_.back();

    // 二分搜尋出夾住 t 的區間 [it-1, it]
    auto it = std::lower_bound(
        nav_state_queue_.begin(), nav_state_queue_.end(), t,
        [](const core::NavState& s, const core::Timestamp& ts) { return s.timestamp < ts; });

    const auto& s1 = *it;
    const auto& s0 = *(it - 1);

    double total = getDeltaSec(s0.timestamp, s1.timestamp);
    if (total < 1e-9) return s0;

    double alpha = std::clamp(getDeltaSec(s0.timestamp, t) / total, 0.0, 1.0);

    // 旋轉 slerp，位置 & 速度線性插值
    Eigen::Quaterniond q0(s0.pose.linear());
    Eigen::Quaterniond q1(s1.pose.linear());

    core::NavState result;
    result.timestamp = t;
    result.pose.linear() = q0.slerp(alpha, q1).normalized().toRotationMatrix();
    result.pose.translation() =
        s0.pose.translation() + alpha * (s1.pose.translation() - s0.pose.translation());
    result.vel = s0.vel + alpha * (s1.vel - s0.vel);
    result.acc_bias = s0.acc_bias;
    result.gyr_bias = s0.gyr_bias;
    result.pose_cov = s0.pose_cov;
    return result;
}

void GtsamImuPreintegrator::initFirstFrame(const gtsam::Pose3& imu_pose) {
    gtsam::ISAM2Params opt_params;
    opt_params.relinearizeThreshold = 0.1;
    opt_params.relinearizeSkip = 1;
    optimizer_ = std::make_unique<gtsam::ISAM2>(opt_params);

    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise =
        gtsam::noiseModel::Isotropic::Sigma(3, 1e4);
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);

    factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), imu_pose, correction_noise_));
    factor_graph_.add(
        gtsam::PriorFactor<gtsam::Vector3>(V(0), gtsam::Vector3(0, 0, 0), prior_vel_noise));
    factor_graph_.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
        B(0), gtsam::imuBias::ConstantBias(), prior_bias_noise));

    initial_graph_values_.insert(X(0), imu_pose);
    initial_graph_values_.insert(V(0), gtsam::Vector3(0, 0, 0));
    initial_graph_values_.insert(B(0), gtsam::imuBias::ConstantBias());

    optimizer_->update(factor_graph_, initial_graph_values_);
    factor_graph_.resize(0);
    initial_graph_values_.clear();

    last_optimized_state_ = gtsam::NavState(imu_pose, gtsam::Vector3(0, 0, 0));
    last_optimized_bias_ = gtsam::imuBias::ConstantBias();

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
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), init_pose, updatedPoseNoise);
    factor_graph_.add(priorPose);
    gtsam::PriorFactor<gtsam::Vector3> priorVel(V(0), init_vel, updatedVelNoise);
    factor_graph_.add(priorVel);
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), init_bias, updatedBiasNoise);
    factor_graph_.add(priorBias);
    initial_graph_values_.insert(X(0), init_pose);
    initial_graph_values_.insert(V(0), init_vel);
    initial_graph_values_.insert(B(0), init_bias);
    optimizer_->update(factor_graph_, initial_graph_values_);
    factor_graph_.resize(0);
    initial_graph_values_.clear();
}

bool GtsamImuPreintegrator::calculateImuBias(const gtsam::Pose3& imu_pose,
                                             const Eigen::Matrix<double, 6, 6>& pose_cov,
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

    const gtsam::PreintegratedImuMeasurements& preint_imu = *imu_integrator_opt_;
    gtsam::ImuFactor imu_factor(X(graph_node_index_ - 1), V(graph_node_index_ - 1),
                                X(graph_node_index_), V(graph_node_index_),
                                B(graph_node_index_ - 1), preint_imu);
    factor_graph_.add(imu_factor);

    gtsam::noiseModel::Diagonal::shared_ptr bias_noise =
        gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
    factor_graph_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        B(graph_node_index_ - 1), B(graph_node_index_), gtsam::imuBias::ConstantBias(),
        gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_integrator_opt_->deltaTij()) *
                                            noise_model_between_bias_)));

    bool is_degenerate = pose_cov.trace() > 1.0;
    factor_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        X(graph_node_index_), imu_pose,
        is_degenerate ? correction_noise_large_ : correction_noise_));

    gtsam::NavState prop_state =
        imu_integrator_opt_->predict(last_optimized_state_, last_optimized_bias_);
    initial_graph_values_.insert(X(graph_node_index_), prop_state.pose());
    initial_graph_values_.insert(V(graph_node_index_), prop_state.velocity());
    initial_graph_values_.insert(B(graph_node_index_), last_optimized_bias_);

    optimizer_->update(factor_graph_, initial_graph_values_);
    optimizer_->update();
    factor_graph_.resize(0);
    initial_graph_values_.clear();

    gtsam::Values result = optimizer_->calculateEstimate();
    last_optimized_state_ = gtsam::NavState(result.at<gtsam::Pose3>(X(graph_node_index_)),
                                            result.at<gtsam::Vector3>(V(graph_node_index_)));
    last_optimized_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(graph_node_index_));

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
    if (ba.norm() > 1.0 || bg.norm() > 1.0) {
        return true;
    }

    return false;
}

void GtsamImuPreintegrator::resetOptimization() {
    gtsam::ISAM2Params optParameters;
    optParameters.relinearizeThreshold = 0.1;
    optParameters.relinearizeSkip = 1;
    optimizer_ = std::make_unique<gtsam::ISAM2>(optParameters);

    factor_graph_.resize(0);
    initial_graph_values_.clear();
}

gtsam::Pose3 GtsamImuPreintegrator::toGtsamPose(const Eigen::Isometry3d& pose) const {
    return gtsam::Pose3(pose.matrix());
}
core::NavState GtsamImuPreintegrator::fromGtsamNavState(const gtsam::NavState& g_state,
                                                        const core::Timestamp& timestamp) const {
    return core::NavState{timestamp, Eigen::Isometry3d(g_state.pose().matrix()),
                          g_state.velocity()};
}

}  // namespace lio_slam_shaw