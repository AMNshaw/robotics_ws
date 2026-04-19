#ifndef LIO_SLAM_SHAW__CORE__I_LIO_INITIALIZER_HPP_
#define LIO_SLAM_SHAW__CORE__I_LIO_INITIALIZER_HPP_

#include <Eigen/Dense>
#include <memory>
#include <optional>

#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

/// Result of LIO initialisation: provides the iEKF with a well-conditioned
/// starting state so that gravity-direction error does not leak into b_a.
struct LioInitResult {
    Timestamp timestamp;                              // time of the last scan used
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // world ← body rotation
    Eigen::Vector3d p = Eigen::Vector3d::Zero();      // position in world
    Eigen::Vector3d v = Eigen::Vector3d::Zero();      // velocity in world
    Eigen::Vector3d b_a = Eigen::Vector3d::Zero();    // accel bias
    Eigen::Vector3d b_g = Eigen::Vector3d::Zero();    // gyro bias
    Eigen::Vector3d gravity = {0.0, 0.0, -9.80511};   // gravity in world frame
};

/// Interface for LiDAR-inertial initialisation.
///
/// Implementations collect the first N LiDAR scans and the corresponding IMU
/// data, then solve for the initial navigation state (R, v, g, biases).
///
/// Example implementations:
///   - SfmLioInitializer : ikd-tree scan-to-map matching + linear alignment
///   - (future) FeatureLioInitializer : edge/planar feature matching
class ILioInitializer {
public:
    using SharedPtr = std::shared_ptr<ILioInitializer>;
    virtual ~ILioInitializer() = default;

    /// Feed a single IMU measurement.  Called from feedImu() on every sample.
    virtual void addImu(const ImuData& imu) = 0;

    /// Feed a deskewed LiDAR scan (after preprocessing / downsampling).
    /// The initialiser performs scan matching internally and buffers the result.
    virtual void addScan(const FeatureSet& features, Timestamp scan_time) = 0;

    /// Returns true when enough scans have been collected and the linear
    /// alignment has been solved successfully.
    virtual bool isReady() const = 0;

    /// Retrieve the solved initial state.  Only valid when isReady() == true.
    virtual LioInitResult getResult() const = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_LIO_INITIALIZER_HPP_
