/// @file test_sfm_lio_initializer.cpp
/// Unit tests for solveLinearAlignment() in SfmLioInitializer.
///
/// Strategy: generate a known trajectory (rotation + constant body-frame
/// acceleration) and synthesise perfect IMU measurements.  Inject the
/// ground-truth poses and IMU buffer directly into the initializer, then
/// verify that solveLinearAlignment() recovers the correct gravity direction
/// and per-frame velocities.
///
/// Because we bypass scan-matching entirely, no point-cloud data is needed.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <cmath>

#include "lio_slam_shaw/initializer/sfm_lio_initializer.hpp"

namespace lio_slam_shaw::initializer {

// ---------------------------------------------------------------------------
// Helper: create a Timestamp from seconds (epoch-based).
// ---------------------------------------------------------------------------
static core::Timestamp tsFromSec(double sec) {
    return core::Timestamp(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(sec)));
}

// ---------------------------------------------------------------------------
// Helper: skew-symmetric matrix.
// ---------------------------------------------------------------------------
static Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(),  //
        v.z(), 0, -v.x(),   //
        -v.y(), v.x(), 0;
    return m;
}

// ---------------------------------------------------------------------------
// Helper: SO(3) exponential (Rodrigues).
// ---------------------------------------------------------------------------
static Eigen::Matrix3d expSO3(const Eigen::Vector3d& phi) {
    const double theta = phi.norm();
    if (theta < 1e-12) return Eigen::Matrix3d::Identity();
    const Eigen::Vector3d a = phi / theta;
    const Eigen::Matrix3d ax = skew(a);
    return Eigen::Matrix3d::Identity() + std::sin(theta) * ax + (1.0 - std::cos(theta)) * (ax * ax);
}

// ===========================================================================
// Test fixture: befriended by SfmLioInitializer so we can touch private
// members (frames_, imu_buf_, solveLinearAlignment(), result_).
// ===========================================================================
class SfmLioInitializerTest : public ::testing::Test {
protected:
    /// Build a minimal SfmLioInitializer (scan-matcher / map-builder will
    /// never be used because we only call solveLinearAlignment()).
    void SetUp() override {
        scan_matcher::IkdTreeScanMatcherParams smp;
        map_builder::IkdTreeLocalMapBuilderParams mbp;
        Eigen::Isometry3d T_imu_lidar = Eigen::Isometry3d::Identity();
        SfmLioInitializerParams params;
        params.min_init_scans = 3;  // irrelevant for this test
        init_ = std::make_shared<SfmLioInitializer>(smp, mbp, T_imu_lidar, params);
    }

    // --- Direct access helpers ---
    auto& frames() { return init_->frames_; }
    auto& imu_buf() { return init_->imu_buf_; }
    bool solveLinearAlignment() { return init_->solveLinearAlignment(); }
    core::LioInitResult result() const { return init_->result_; }

    std::shared_ptr<SfmLioInitializer> init_;

    // -----------------------------------------------------------------------
    // Trajectory generator: produces ground-truth frames + IMU data.
    //
    //   - Gravity in world frame: g_world  (user-specified, e.g. tilted)
    //   - Body-frame angular velocity: omega_body (constant)
    //   - Body-frame specific force (accelerometer reading): f_body (constant)
    //   - IMU rate: imu_hz
    //   - Frame rate: frame_hz
    //   - Duration: duration_sec
    //
    // The IMU measurement model:
    //   acc_meas = R^T (a_world - g_world)     (body frame)
    //            = f_body                       (since a_world = R * f_body + g_world)
    //   gyr_meas = omega_body
    //
    // World-frame dynamics:
    //   a_world = R * f_body + g_world
    //   v(t+dt) = v(t) + a_world * dt
    //   p(t+dt) = p(t) + v(t)*dt + 0.5*a_world*dt^2
    //   R(t+dt) = R(t) * Exp(omega_body * dt)
    // -----------------------------------------------------------------------
    struct TrajectoryParams {
        Eigen::Vector3d g_world{0.0, 0.0, -9.80511};
        Eigen::Vector3d omega_body{0.0, 0.0, 0.05};  // slow yaw
        Eigen::Vector3d f_body{0.3, 0.0, 9.80511};   // forward accel + anti-gravity
        Eigen::Vector3d v0{0.0, 0.0, 0.0};           // initial velocity
        double imu_hz = 200.0;
        double frame_hz = 10.0;
        double duration_sec = 1.0;
    };

    struct TrajectoryData {
        std::vector<SfmLioInitializer::ScanFrame> gt_frames;
        std::deque<core::ImuData> imu_data;
        std::vector<Eigen::Vector3d> gt_velocities;  // per-frame velocity (world)
        Eigen::Vector3d g_world;
    };

    static TrajectoryData generateTrajectory(const TrajectoryParams& tp) {
        TrajectoryData td;
        td.g_world = tp.g_world;

        const double imu_dt = 1.0 / tp.imu_hz;
        const double frame_dt = 1.0 / tp.frame_hz;
        const int total_imu_steps = static_cast<int>(std::round(tp.duration_sec * tp.imu_hz));
        const int total_frames = static_cast<int>(std::round(tp.duration_sec * tp.frame_hz)) + 1;

        // Initial state
        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Vector3d v = tp.v0;
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        double t = 0.0;

        // Record first frame
        {
            SfmLioInitializer::ScanFrame sf;
            sf.time = tsFromSec(t);
            sf.pose = Eigen::Isometry3d::Identity();
            sf.pose.linear() = R;
            sf.pose.translation() = p;
            td.gt_frames.push_back(sf);
            td.gt_velocities.push_back(v);
        }

        int next_frame_idx = 1;
        double next_frame_time = frame_dt;

        for (int step = 0; step < total_imu_steps; ++step) {
            // IMU at time t
            core::ImuData imu;
            imu.timestamp = tsFromSec(t);
            imu.acc = tp.f_body;      // constant body-frame specific force
            imu.gyr = tp.omega_body;  // constant body-frame angular velocity
            td.imu_data.push_back(imu);

            // Integrate one step
            const Eigen::Vector3d a_world = R * tp.f_body + tp.g_world;
            p = p + v * imu_dt + 0.5 * a_world * imu_dt * imu_dt;
            v = v + a_world * imu_dt;
            R = R * expSO3(tp.omega_body * imu_dt);
            t += imu_dt;

            // Record frame if at frame boundary
            if (next_frame_idx < total_frames && t >= next_frame_time - 1e-9) {
                SfmLioInitializer::ScanFrame sf;
                sf.time = tsFromSec(t);
                sf.pose = Eigen::Isometry3d::Identity();
                sf.pose.linear() = R;
                sf.pose.translation() = p;
                td.gt_frames.push_back(sf);
                td.gt_velocities.push_back(v);
                ++next_frame_idx;
                next_frame_time = next_frame_idx * frame_dt;
            }
        }

        // Final IMU sample at time t (needed for boundary interpolation)
        core::ImuData imu_final;
        imu_final.timestamp = tsFromSec(t);
        imu_final.acc = tp.f_body;
        imu_final.gyr = tp.omega_body;
        td.imu_data.push_back(imu_final);

        return td;
    }
};

// ===========================================================================
// Test 1: Constant velocity, no rotation (simplest case)
// Gravity = [0, 0, -9.80511], zero angular velocity, f_body = pure anti-gravity
// ===========================================================================
TEST_F(SfmLioInitializerTest, ConstantVelocity_NoRotation) {
    TrajectoryParams tp;
    tp.g_world = {0.0, 0.0, -9.80511};
    tp.omega_body = {0.0, 0.0, 0.0};
    tp.f_body = {0.0, 0.0, 9.80511};  // cancels gravity → zero world accel
    tp.v0 = {1.0, 0.5, 0.0};          // constant velocity
    tp.imu_hz = 500.0;
    tp.frame_hz = 10.0;
    tp.duration_sec = 1.0;

    const auto td = generateTrajectory(tp);

    // Inject directly
    frames() = td.gt_frames;
    imu_buf() = td.imu_data;

    ASSERT_TRUE(solveLinearAlignment());

    const auto res = result();

    // Gravity should be [0, 0, -9.80511] after alignment
    EXPECT_NEAR(res.gravity.x(), 0.0, 0.01);
    EXPECT_NEAR(res.gravity.y(), 0.0, 0.01);
    EXPECT_NEAR(res.gravity.z(), -9.80511, 0.01);

    // Velocity of last frame (rotated to g-frame) should match ground truth
    // In this case g_world is already aligned, so R_gw ≈ I
    const auto& gt_v = td.gt_velocities.back();
    // After gravity alignment rotation, the velocity should be close
    EXPECT_NEAR(res.v.norm(), gt_v.norm(), 0.05);

    std::cout << "[ConstVel] g_solved = " << res.gravity.transpose()
              << " |g| = " << res.gravity.norm() << "\n";
    std::cout << "[ConstVel] v_last   = " << res.v.transpose() << " gt = " << gt_v.transpose()
              << "\n";
}

// ===========================================================================
// Test 2: Accelerating with rotation (more realistic)
// Body-frame has forward acceleration + slow yaw
// ===========================================================================
TEST_F(SfmLioInitializerTest, AcceleratingWithRotation) {
    TrajectoryParams tp;
    tp.g_world = {0.0, 0.0, -9.80511};
    tp.omega_body = {0.0, 0.0, 0.1};  // 0.1 rad/s yaw
    tp.f_body = {0.5, 0.0, 9.80511};  // 0.5 m/s^2 forward + anti-gravity
    tp.v0 = {0.0, 0.0, 0.0};
    tp.imu_hz = 500.0;
    tp.frame_hz = 10.0;
    tp.duration_sec = 1.5;

    const auto td = generateTrajectory(tp);
    ASSERT_GE(td.gt_frames.size(), 10u);

    frames() = td.gt_frames;
    imu_buf() = td.imu_data;

    ASSERT_TRUE(solveLinearAlignment());

    const auto res = result();

    // Gravity direction: after alignment should be [0, 0, -G]
    EXPECT_NEAR(res.gravity.x(), 0.0, 0.02);
    EXPECT_NEAR(res.gravity.y(), 0.0, 0.02);
    EXPECT_NEAR(res.gravity.z(), -9.80511, 0.02);
    EXPECT_NEAR(res.gravity.norm(), 9.80511, 1e-4);

    std::cout << "[AccelRot] g_solved = " << res.gravity.transpose()
              << " |g| = " << res.gravity.norm() << "\n";
    std::cout << "[AccelRot] v_last   = " << res.v.transpose() << "\n";
}

// ===========================================================================
// Test 3: Tilted gravity (sensor mounted on a slope)
// Gravity is NOT aligned with world Z.  The solver must recover the correct
// direction and the final R_gw rotation must align it to [0, 0, -G].
// ===========================================================================
TEST_F(SfmLioInitializerTest, TiltedGravity) {
    // Tilt gravity 15 degrees around Y axis
    const double tilt = 15.0 * M_PI / 180.0;
    const Eigen::Vector3d g_tilted =
        Eigen::AngleAxisd(tilt, Eigen::Vector3d::UnitY()).toRotationMatrix() *
        Eigen::Vector3d(0.0, 0.0, -9.80511);

    TrajectoryParams tp;
    tp.g_world = g_tilted;
    tp.omega_body = {0.0, 0.0, 0.0};
    tp.f_body = -g_tilted;  // no world-frame acceleration → straight line
    // Actually, f_body = R^T * (a_world - g) = R^T * (0 - g) = -R^T * g
    // For R=I at start, f_body = -g_tilted. But R changes if omega ≠ 0.
    // Since omega = 0, R stays I, so f_body = -g_tilted throughout.
    tp.v0 = {0.5, -0.3, 0.1};
    tp.imu_hz = 500.0;
    tp.frame_hz = 10.0;
    tp.duration_sec = 1.0;

    const auto td = generateTrajectory(tp);

    frames() = td.gt_frames;
    imu_buf() = td.imu_data;

    ASSERT_TRUE(solveLinearAlignment());

    const auto res = result();

    // After gravity alignment, gravity should be [0, 0, -G] regardless of input tilt
    EXPECT_NEAR(res.gravity.x(), 0.0, 0.02);
    EXPECT_NEAR(res.gravity.y(), 0.0, 0.02);
    EXPECT_NEAR(res.gravity.z(), -9.80511, 0.02);

    std::cout << "[Tilted]   g_input  = " << g_tilted.transpose() << "\n";
    std::cout << "[Tilted]   g_solved = " << res.gravity.transpose()
              << " |g| = " << res.gravity.norm() << "\n";
}

// ===========================================================================
// Test 4: Too few frames → should return false
// ===========================================================================
TEST_F(SfmLioInitializerTest, TooFewFrames_ReturnsFalse) {
    // Only 2 frames → 1 interval → N < 2 → should fail
    SfmLioInitializer::ScanFrame f0, f1;
    f0.time = tsFromSec(0.0);
    f0.pose = Eigen::Isometry3d::Identity();
    f1.time = tsFromSec(0.1);
    f1.pose = Eigen::Isometry3d::Identity();
    f1.pose.translation() = Eigen::Vector3d(0.1, 0, 0);

    frames() = {f0, f1};

    // Add some IMU
    for (int i = 0; i <= 50; ++i) {
        core::ImuData imu;
        imu.timestamp = tsFromSec(i * 0.002);
        imu.acc = Eigen::Vector3d(0, 0, 9.8);
        imu.gyr = Eigen::Vector3d::Zero();
        imu_buf().push_back(imu);
    }

    EXPECT_FALSE(solveLinearAlignment());
}

// ===========================================================================
// Test 5: Full rotation + acceleration (circular arc)
// Tests that the solver handles significant rotation correctly.
// ===========================================================================
TEST_F(SfmLioInitializerTest, CircularArc) {
    TrajectoryParams tp;
    tp.g_world = {0.0, 0.0, -9.80511};
    tp.omega_body = {0.0, 0.0, 0.5};  // 0.5 rad/s yaw → ~57 deg over 2s
    tp.f_body = {1.0, 0.0, 9.80511};  // forward + anti-gravity
    tp.v0 = {0.0, 0.0, 0.0};
    tp.imu_hz = 500.0;
    tp.frame_hz = 10.0;
    tp.duration_sec = 2.0;

    const auto td = generateTrajectory(tp);
    ASSERT_GE(td.gt_frames.size(), 15u);

    frames() = td.gt_frames;
    imu_buf() = td.imu_data;

    ASSERT_TRUE(solveLinearAlignment());

    const auto res = result();

    // Gravity direction
    const Eigen::Vector3d g_dir = res.gravity.normalized();
    EXPECT_NEAR(g_dir.x(), 0.0, 0.03);
    EXPECT_NEAR(g_dir.y(), 0.0, 0.03);
    EXPECT_NEAR(g_dir.z(), -1.0, 0.03);
    EXPECT_NEAR(res.gravity.norm(), 9.80511, 1e-3);

    std::cout << "[CircArc]  g_solved = " << res.gravity.transpose()
              << " |g| = " << res.gravity.norm() << "\n";
    std::cout << "[CircArc]  v_last   = " << res.v.transpose() << "\n";
}

}  // namespace lio_slam_shaw::initializer
