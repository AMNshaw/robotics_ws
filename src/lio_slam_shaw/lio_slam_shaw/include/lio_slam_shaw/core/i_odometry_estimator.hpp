#ifndef LIO_SLAM_SHAW__CORE__I_ODOMETRY_ESTIMATOR_HPP_
#define LIO_SLAM_SHAW__CORE__I_ODOMETRY_ESTIMATOR_HPP_

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/i_lio_initializer.hpp"
#include "lio_slam_shaw/core/sensor_data_types.hpp"

namespace lio_slam_shaw::core {

struct OdometryResult {
    ScanMatchResult matched_in_map;
    ScanMatchResult matched_in_odom;
    NavState corrected_state;
};

class IOdometryEstimator {
public:
    using SharedPtr = std::shared_ptr<IOdometryEstimator>;
    virtual ~IOdometryEstimator() = default;

    // Called on every incoming IMU message. The estimator buffers IMU data
    // internally and uses it to split opt_batch / reprop_batch at scan time.
    virtual void feedImu(const ImuData& imu) = 0;

    // Called once per LiDAR scan, after deskew and feature extraction.
    // lidar_time_start is used to partition the internal IMU buffer into:
    //   opt_batch   : [prev_scan_time, lidar_time_start)  — for bias update
    //   reprop_batch: [lidar_time_start, latest_imu_time) — for re-propagation
    virtual OdometryResult estimateWithFeatures(const FeatureSet& features,
                                                Timestamp lidar_time_start) = 0;

    // Latest IMU-predicted state; used by FrontEnd to seed the deskew nav snapshot.
    virtual NavState getLatestState() const = 0;

    /// Return a chronological snapshot of predicted states (for deskewing).
    virtual std::vector<NavState> getNavStateQueueSnapshot() const = 0;

    /// Apply the result from LIO initialisation (gravity, velocity, pose, biases)
    /// to the internal state.  Called once when the initialiser becomes ready.
    virtual void setInitialState(const LioInitResult& init_result) = 0;
};

}  // namespace lio_slam_shaw::core

#endif  // LIO_SLAM_SHAW__CORE__I_ODOMETRY_ESTIMATOR_HPP_
