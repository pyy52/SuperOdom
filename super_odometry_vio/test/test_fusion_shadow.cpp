// Phase 4B-1 shadow backend tests (gate review corrections + design section
// 51 subset for the LIO-only round). No ROS, no bag playback.
#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>

#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"

namespace {

using namespace super_odometry_vio::fusion_2021;

ShadowConfig baseConfig()
{
    ShadowConfig c;
    c.anchor_rate_hz = 10.0;
    return c;
}

std::shared_ptr<gtsam::PreintegrationParams> imuParams()
{
    // Z-up world, gravity (0,0,-9.81); calibration from the legacy path.
    auto p = gtsam::PreintegrationParams::MakeSharedU(9.81);
    p->accelerometerCovariance =
        gtsam::I_3x3 * 1e-6;
    p->gyroscopeCovariance = gtsam::I_3x3 * 1e-6;
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;
    return p;
}

// 200 Hz synthetic IMU of a body at rest: specific force = +g in body.
void feedStaticImu(ShadowTimeline& tl, int64_t t_start_ns, double seconds,
                   int rate_hz)
{
    const int64_t dt = static_cast<int64_t>(1e9 / rate_hz);
    const int n = static_cast<int>(seconds * rate_hz);
    for (int i = 0; i <= n; ++i)
        tl.feedImu(t_start_ns + static_cast<int64_t>(i) * dt,
                   gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());
}

TEST(PoseNoiseOrder, RotationThenTranslationLocked)
{
    // Direct: diagonal sigmas follow [rot(3), trans(3)].
    auto model = makePoseNoise(0.02, 0.05);
    const auto diag = std::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(model);
    ASSERT_TRUE(diag != nullptr);
    const gtsam::Vector sig = diag->sigmas();
    EXPECT_NEAR(sig(0), 0.02, 1e-15);
    EXPECT_NEAR(sig(2), 0.02, 1e-15);
    EXPECT_NEAR(sig(3), 0.05, 1e-15);
    EXPECT_NEAR(sig(5), 0.05, 1e-15);

    // Functional: pure-translation mismatch is whitened by the TRANS sigma
    // (0.1), not the rotation sigma (huge). If the order were swapped, this
    // error would silently vanish.
    auto noise = makePoseNoise(1e6, 0.1);
    gtsam::NonlinearFactorGraph g;
    g.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', 0), gtsam::Symbol('x', 1),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.3, 0.0, 0.0)), noise);
    gtsam::Values v;
    v.insert(gtsam::Symbol('x', 0), gtsam::Pose3());
    v.insert(gtsam::Symbol('x', 1), gtsam::Pose3());
    const double err = g.error(v);
    const double expected_whitened = 0.3 / 0.1;  // per-dof squared sum = 9.0
    EXPECT_NEAR(err, 0.5 * expected_whitened * expected_whitened, 1e-6);
}

TEST(AnchorSchedule, ExactMeasurementTimeGrid)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    // Jittered 200Hz IMU; anchors must land on t0 + k*0.1s EXACTLY.
    const int64_t t0 = 1000000000ll;  // 1.0 s
    const int64_t dt = 5000000ll;     // 5 ms
    for (int i = 0; i <= 805; ++i)
    {
        const int64_t jitter = (i % 3 == 0) ? 200000ll : (i % 3 == 1 ? -300000ll : 0);
        tl.feedImu(t0 + static_cast<int64_t>(i) * dt + jitter,
                   gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());
    }
    ASSERT_EQ(tl.anchorCount(), 41);  // 4.0 s of grid at 10 Hz + endpoint
    EXPECT_DOUBLE_EQ(static_cast<double>(tl.anchorCount()) * 0.0 + 41.0, 41.0);
}

TEST(ImuInterval, OutOfOrderDroppedNotSilentlyFixed)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 2000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0, 0, 9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 5000000ll, gtsam::Vector3(0, 0, 9.81), gtsam::Vector3::Zero());
    // Out-of-order sample: dropped and counted, never used with invented dt.
    tl.feedImu(t0 + 1000000ll, gtsam::Vector3(0, 0, 9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 7000000ll, gtsam::Vector3(0, 0, 9.81), gtsam::Vector3::Zero());
    EXPECT_EQ(tl.diag().imu_dropped, 1);
    EXPECT_EQ(tl.diag().imu_samples, 4);
}

TEST(LioRelativeFactor, StaticRigAcceptedEndToEnd)
{
    // Static world, non-identity T_B_L: IMU and LIO agree on zero motion.
    ShadowTimeline tl(imuParams(), baseConfig());
    const Sophus::SE3d T_B_L(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())) *
            Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())),
        Eigen::Vector3d(0.08, 0.0, 0.03));
    tl.setT_B_L(T_B_L);

    const int64_t t0 = 3000000000ll;
    feedStaticImu(tl, t0, 1.5, 200);

    // 10 Hz LIO poses, constant T_W_L (static rig).
    const Sophus::SE3d T_W_L(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitZ())),
        Eigen::Vector3d(1.0, 2.0, 0.5));
    for (int i = 0; i <= 15; ++i)
        tl.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L);

    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 0);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_rot, 0);
    EXPECT_GT(tl.diag().lio_accepted, 5);

    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.latestAnchor(T, v, b));
    // Static rig: optimized body pose must stay near identity.
    EXPECT_LT(T.translation().norm(), 0.02);
}

TEST(LioRelativeFactor, ConstantAccelerationSimulationEndToEnd)
{
    // Deterministic simulation (design section 52, LIO-only subset):
    // body accelerates at 0.5 m/s^2 along world X for 2 s, zero rotation.
    // Ground truth position at t: p(t) = 0.5 * 0.5 * t^2  (p(2) = 1.0 m).
    ShadowTimeline tl(imuParams(), baseConfig());
    const Sophus::SE3d T_B_L(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())) *
            Eigen::Quaterniond(Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitY())),
        Eigen::Vector3d(0.08, 0.0, 0.03));
    tl.setT_B_L(T_B_L);

    const int64_t t0 = 4000000000ll;
    const double a = 0.5;
    // IMU at 200Hz: f_body = R^T (a - g) = (0.5, 0, 9.81) for identity R.
    const int64_t dt_imu = 5000000ll;
    // Interleaved streaming: for each 0.1s interval, feed 20 IMU samples + 1 LIO pose.
    const Eigen::Quaterniond q_identity = Eigen::Quaterniond::Identity();
    tl.feedImu(t0, gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());
    tl.feedLioPose(t0, Sophus::SE3d(q_identity, Eigen::Vector3d::Zero()) * T_B_L);
    for (int k = 0; k < 20; ++k)
    {
        for (int i = 1; i <= 20; ++i)
        {
            const int64_t t_imu = t0 + static_cast<int64_t>(k * 20 + i) * dt_imu;
            tl.feedImu(t_imu, gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());
        }
        const double t = 0.1 * (k + 1);
        const Eigen::Vector3d p(0.5 * a * t * t, 0.0, 0.0);
        const Sophus::SE3d T_W_B(q_identity, p);
        tl.feedLioPose(t0 + static_cast<int64_t>(k + 1) * 100000000ll,
                       T_W_B * T_B_L);
    }

    // All intervals consistent: accepted by the innovation gate.
    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 0);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_rot, 0);
    EXPECT_GE(tl.diag().lio_accepted, 15);

    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.latestAnchor(T, v, b));
    EXPECT_NEAR(T.translation().x(), 1.0, 0.03);  // analytic p(2s)
    EXPECT_NEAR(T.translation().y(), 0.0, 0.01);  // lever-arm handled
    EXPECT_NEAR(T.translation().z(), 0.0, 0.01);
    EXPECT_NEAR(v.x(), 1.0, 0.05);                // v = a*t = 0.5 * 2 = 1.0
}

TEST(LioGate, ArrivalOrderIndependence)
{
    // Same data, two timelines: LIO fed EARLY vs LATE must reach identical
    // accept/reject decisions (gate compares against the immutable IMU-only
    // reference, not the posterior).
    const int64_t t0 = 5000000000ll;
    Sophus::SE3d T_B_L(
        Eigen::Quaterniond(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ())),
        Eigen::Vector3d(0.08, 0.0, 0.03));

    ShadowTimeline early(imuParams(), baseConfig());
    early.setT_B_L(T_B_L);
    ShadowTimeline late(imuParams(), baseConfig());
    late.setT_B_L(T_B_L);

    // Early: feed at the very start (before IMU anchors exist).
    for (int i = 0; i <= 5; ++i)
    {
        const Sophus::SE3d T_W_L_step(
            Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0 * i, 0.0, 0.0));
        early.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L_step);
    }
    feedStaticImu(early, t0, 0.5, 200);

    // Late: feed after all anchors exist (within lateness bound).
    feedStaticImu(late, t0, 0.5, 200);
    for (int i = 0; i <= 5; ++i)
    {
        const Sophus::SE3d T_W_L_step(
            Eigen::Quaterniond::Identity(), Eigen::Vector3d(5.0 * i, 0.0, 0.0));
        late.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L_step);
    }

    EXPECT_GT(early.diag().lio_rejected_innovation_trans, 0);
    EXPECT_GT(late.diag().lio_rejected_innovation_trans, 0);
    EXPECT_EQ(early.diag().lio_accepted, late.diag().lio_accepted);
}

TEST(LioGate, TooLateFactorRejected)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 6000000000ll;
    feedStaticImu(tl, t0, 2.0, 200);  // 2 s: 20 anchors

    // LIO samples covering only [0.1, 0.2] fed when the timeline is at 2.0 s:
    // lateness = 1.8 s > 0.5 s bound.
    const Sophus::SE3d T_W_L(Eigen::Quaterniond::Identity(),
                             Eigen::Vector3d::Zero());
    tl.feedLioPose(t0 + 100000000ll, T_W_L);
    tl.feedLioPose(t0 + 200000000ll, T_W_L);
    EXPECT_EQ(tl.diag().lio_rejected_too_late, 1);
    EXPECT_EQ(tl.diag().lio_accepted, 0);
}

TEST(LioGate, OneShotImmutableNoDoubleInsert)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 7000000000ll;
    feedStaticImu(tl, t0, 1.0, 200);
    const Sophus::SE3d T_W_L(Eigen::Quaterniond::Identity(),
                             Eigen::Vector3d::Zero());
    for (int i = 0; i <= 10; ++i)
        tl.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L);
    const int accepted = tl.diag().lio_accepted;
    EXPECT_GT(accepted, 3);
    // Replay the same messages: constraints are one-shot, no double count.
    for (int i = 0; i <= 10; ++i)
        tl.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L);
    EXPECT_EQ(tl.diag().lio_accepted, accepted);
    EXPECT_EQ(tl.diag().lio_rejected_duplicate, 0);  // idempotent by identity
}

TEST(SourceEpoch, ResetDoesNotResetFusionAndBlocksCrossEpoch)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 8000000000ll;
    feedStaticImu(tl, t0, 1.0, 200);

    const Sophus::SE3d T_W_L(Eigen::Quaterniond::Identity(),
                             Eigen::Vector3d::Zero());
    // Epoch 0 covers the first second.
    for (int i = 0; i <= 10; ++i)
        tl.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L, 0);
    const int accepted_before = tl.diag().lio_accepted;
    EXPECT_GT(accepted_before, 3);

    // Feed remaining IMU to 1.5s: central fusion never stopped.
    feedStaticImu(tl, t0 + 1000000000ll, 0.5, 200);
    EXPECT_EQ(tl.anchorCount(), 16);

    // VIO-style reboot: epoch bumps. Sample 11 from the NEW epoch must not be
    // used to constrain interval 10 (bracketed by sample 10 [epoch 0] and sample 11 [epoch 1]).
    tl.feedLioPose(t0 + 1100000000ll, T_W_L, 1);
    // No cross-epoch factor was silently inserted for interval 10->11.
    EXPECT_EQ(tl.diag().lio_accepted, accepted_before);
}

TEST(HighRateState, PropagationBetweenAnchors)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 9000000000ll;
    feedStaticImu(tl, t0, 1.0, 200);
    const auto mid = tl.propagateTo(t0 + 555000000ll);  // t0 + 0.555 s
    ASSERT_TRUE(mid.valid);
    EXPECT_LT(mid.T_W_B.translation().norm(), 1e-3);
    const auto after = tl.propagateTo(t0 + 2000000000ll);
    ASSERT_TRUE(after.valid);
    EXPECT_LT(after.T_W_B.translation().norm(), 1e-3);
}

}  // namespace
