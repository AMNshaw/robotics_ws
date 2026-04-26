#ifndef LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_
#define LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <vector>

#include "lio_slam_shaw/core/i_lio_initializer.hpp"
#include "lio_slam_shaw/map_builder/ikd_tree_map_builder.hpp"
#include "lio_slam_shaw/scan_matcher/ikd_tree_scan_matcher.hpp"

namespace lio_slam_shaw::initializer {

struct SfmLioInitializerParams {
    int min_init_scans = 10;  // minimum scans before attempting linear alignment

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
                               const map_builder::IkdTreeMapBuilderParams& map_builder_params,
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

private:
    struct BufferedScan {
        core::Timestamp time;
        core::PointCloudIRTPtr cloud;  // downsampled
    };

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
    /// Insert a scan's points into the ikd-tree map (transformed to world frame).
    void insertScanToMap(const core::FeatureSet& features, const Eigen::Isometry3d& T_world_lidar);

    // --- IMU preintegration (simplified, no covariance) ---
    PreintResult preintegrateImu(const std::vector<core::ImuData>& imu_batch) const;

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
    map_builder::IkdTreeMapBuilderParams map_builder_params_;
    scan_matcher::IkdTreeScanMatcherParams scan_matcher_params_;
    std::shared_ptr<map_builder::IkdTreeMapBuilder> map_builder_;
    std::shared_ptr<scan_matcher::IkdTreeScanMatcher> scan_matcher_;
    Eigen::Isometry3d T_imu_lidar_;

    // Buffered raw scans (before scan matching)

    std::vector<BufferedScan> scan_buf_;

    // Buffered scan frames

    std::vector<ScanFrame> frames_;

    // IMU buffer: all IMU samples since the start
    std::deque<core::ImuData> imu_buf_;

    // Result
    bool ready_ = false;
    core::LioInitResult result_;
};

}  // namespace lio_slam_shaw::initializer

#endif  // LIO_SLAM_SHAW__INITIALIZER__SFM_LIO_INITIALIZER_HPP_
