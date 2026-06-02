#ifndef LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_
#define LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_

#include <Eigen/Dense>
#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>

#include "lio_slam_shaw/core/i_lio_initializer.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_local_map_builder.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::initializer {

struct SfmLioInitializerParams {
    int min_init_scans = 10;  // minimum scans before attempting linear alignment

    // If true, rotate the LiDAR SFM frame to the gravity direction solved from
    // short-window IMU alignment. For datasets whose base_link/velodyne frame
    // is already z-up, a noisy short-window gravity estimate can introduce a
    // large artificial map tilt, so keep this disabled by default.
    bool align_gravity = false;

    // Voxel downsample leaf size for raw cloud (0 = no downsample)
    float voxel_leaf_size = 0.5f;

    // IMU noise (for preintegration weighting in linear solve)
    double acc_noise = 3.9939570888117902e-03;
    double gyr_noise = 1.5636343949698187e-03;
};

/// Multi-frame LIO initialiser using scan-to-map SFM + IMU linear alignment.
///
/// Phase 1 (scan-by-scan): Each incoming scan is matched against the ikd-tree
///   map using Gauss-Newton point-to-plane ICP (6-DOF pose, no IMU).
///   The first scan sets the origin; subsequent scans use the previous pose as
///   initial guess.
///
/// Phase 2 (linear alignment): After min_init_scans frames, uses the pose
///   sequence + IMU preintegration between frames to solve a linear system
///   for {v_0..v_N, gravity} (VINS-Mono style).
///
/// Phase 3 (refinement): Optionally refines b_a and b_g from the solved
///   gravity direction.
class SfmLioInitializer : public core::ILioInitializer {
public:
    using SharedPtr = std::shared_ptr<SfmLioInitializer>;

    explicit SfmLioInitializer(const scan_matcher::IkdTreeScanMatcherParams& scan_matcher_params,
                               const map_builder::IkdTreeLocalMapBuilderParams& map_builder_params,
                               const Eigen::Isometry3d& T_imu_lidar,
                               const SfmLioInitializerParams& params = {});
    ~SfmLioInitializer() override = default;

    void addImu(const core::ImuData& imu) override;
    void addScan(const core::LidarData& lidar) override;
    bool hasEnoughData() const override;
    bool tryInitialize() override;
    bool isReady() const override;
    core::LioInitResult getResult() const override;
    void clearScans() override;
    const std::deque<core::ImuData>& getImuBuffer() const override { return imu_buf_; }

    // Public data types (pure POD, no encapsulation concern)
    struct ScanFrame {
        core::Timestamp time;
        Eigen::Isometry3d pose;  // T_world_body (from scan matching)
    };

    struct PreintResult {
        double dt = 0.0;
        Eigen::Vector3d alpha = Eigen::Vector3d::Zero();  // position increment (body frame)
        Eigen::Vector3d beta = Eigen::Vector3d::Zero();   // velocity increment (body frame)
        Eigen::Matrix3d delta_R = Eigen::Matrix3d::Identity();
    };

private:
    struct BufferedScan {
        core::Timestamp time;          // scan reference timestamp
        core::PointCloudIRTPtr cloud;  // downsampled
    };
    /// Insert a scan's points into the ikd-tree map (transformed to world frame).
    void insertScanToMap(const core::FeatureSet& features, const Eigen::Isometry3d& T_world_lidar);

    // --- IMU preintegration (simplified, no covariance) ---
    /// Integrate IMU batch with an optional gyro-bias correction (subtracted from raw gyr).
    PreintResult preintegrateImu(const std::vector<core::ImuData>& imu_batch,
                                 const Eigen::Vector3d& b_g = Eigen::Vector3d::Zero()) const;

    // --- Gyro bias estimation from rotation residuals ---
    /// Given scan-match relative rotations and raw preint results (b_g=0),
    /// solve for b_g via least-squares: dt * b_g ≈ Log(R_scan^T * R_imu)
    Eigen::Vector3d estimateGyroBias(const std::vector<PreintResult>& preints) const;

    // --- Linear alignment ---
    /// Solve for {v_0, v_1, ..., v_N, gravity} given N+1 poses and N preint results.
    /// Returns true if the solution is valid (gravity magnitude check).
    bool solveLinearAlignment();

    Eigen::Vector3d refineGravity(const std::vector<ScanFrame>& frames,
                                  const std::vector<PreintResult>& preints,
                                  const Eigen::Vector3d& g_solved,
                                  std::vector<Eigen::Vector3d>& velocities);

    // --- State ---
    SfmLioInitializerParams params_;
    map_builder::IkdTreeLocalMapBuilderParams map_builder_params_;
    scan_matcher::IkdTreeScanMatcherParams scan_matcher_params_;
    std::shared_ptr<map_builder::IkdTreeLocalMapBuilder> map_builder_;
    std::shared_ptr<scan_matcher::IkdTreeScanMatcher> scan_matcher_;
    Eigen::Isometry3d T_imu_lidar_;

    // Buffered raw scans (before scan matching)
    mutable std::mutex scan_buf_mutex_;
    std::vector<BufferedScan> scan_buf_;
    std::atomic<bool> initializing_{false};

    // Buffered scan frames
    std::vector<ScanFrame> frames_;

    // IMU buffer: all IMU samples since the start
    std::deque<core::ImuData> imu_buf_;

    // Result
    bool ready_ = false;
    core::LioInitResult result_;

    // Allow unit tests to directly inject frames_/imu_buf_ and call solveLinearAlignment()
    friend class SfmLioInitializerTest;
};

}  // namespace lio_slam_shaw::initializer

#endif  // LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_
