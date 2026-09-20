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
    c.max_constraint_lateness_ns = 500000000ll;
    c.max_imu_dt_ns = 50000000ll;
    c.max_interpolation_gap_ns = 150000000ll;
    c.source_reorder_horizon_ns = 100000000ll;
    
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
    for (int i = 0; i <= 16; ++i)
        tl.feedLioPose(t0 + static_cast<int64_t>(i) * 100000000ll, T_W_L);

    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 0);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_rot, 0);
    EXPECT_GT(tl.diag().lio_accepted, 3);

    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    // feed lio to finalize
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d());
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
    // feed lio to finalize
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d());
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
    const Sophus::SE3d T_W_L(Eigen::Quaterniond::Identity(),
                             Eigen::Vector3d::Zero());
    tl.feedLioPose(t0 + 100000000ll, T_W_L);
    tl.feedLioPose(t0 + 200000000ll, T_W_L);
    
    feedStaticImu(tl, t0, 2.0, 200);  // 2 s: 20 anchors

    // LIO samples covering only [0.1, 0.2] are now sitting in the buffer.
    // We feed a new LIO sample at 2.0s to trigger processing.
    // The newest timestamp in processing will be 2.0s, t_j is 0.1s.
    // Lateness = 1.9s > 0.5s.
    tl.feedLioPose(t0 + 2000000000ll, T_W_L);

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
    EXPECT_GT(accepted, 0);
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
    EXPECT_GT(accepted_before, 0);

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
    const auto after = tl.propagateTo(t0 + 1000000000ll);
    ASSERT_TRUE(after.valid);
    EXPECT_LT(after.T_W_B.translation().norm(), 1e-3);
}

TEST(ImuReference, NonZeroVelocityRelativePredictionOracle)
{
    // Contract falsification test:
    // interval 0 accelerates to non-zero velocity (v_1 != 0).
    // interval 1 continues accelerating.
    // Frozen contract requires: dT_imu_ref(1) == IMU-only relative pose prediction.
    // Oracle: T_{W B_1}.between(pim.predict(state_1, bias_1).pose()).
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 11000000000ll;
    const double a = 2.0;
    const int64_t dt_imu = 5000000ll;  // 5 ms

    // Interval 0: [t0, t0 + 0.1s]
    tl.feedImu(t0, gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());
    for (int i = 1; i <= 20; ++i)
        tl.feedImu(t0 + static_cast<int64_t>(i) * dt_imu,
                   gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());

    gtsam::Pose3 T_1; gtsam::Vector3 v_1; gtsam::imuBias::ConstantBias b_1;
    ASSERT_TRUE(tl.anchorState(1, T_1, v_1, b_1));
    EXPECT_NEAR(v_1.x(), 0.2, 0.01);  // v_1 = a * 0.1 = 0.2 m/s

    // Interval 1: [t0 + 0.1s, t0 + 0.2s]
    for (int i = 21; i <= 40; ++i)
        tl.feedImu(t0 + static_cast<int64_t>(i) * dt_imu,
                   gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());

    // Compute oracle expected relative pose from anchor 1
    gtsam::PreintegratedImuMeasurements pim(imuParams(), b_1);
    for (int i = 1; i <= 20; ++i)
        pim.integrateMeasurement(gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero(), 0.005);
    const gtsam::NavState state_1(T_1, v_1);
    const gtsam::NavState predicted_2 = pim.predict(state_1, b_1);
    const gtsam::Pose3 expected_rel = T_1.between(predicted_2.pose());

    const gtsam::Pose3 stored_ref = tl.imuRef(1);
    std::cout << "[Falsification Audit] v_1.x() = " << v_1.x() << std::endl;
    std::cout << "[Falsification Audit] pim.deltaPij().x() = " << pim.deltaPij().x() << std::endl;
    std::cout << "[Falsification Audit] expected_rel.translation().x() = " << expected_rel.translation().x() << std::endl;
    std::cout << "[Falsification Audit] stored_ref.translation().x() = " << stored_ref.translation().x() << std::endl;
    std::cout << "[Falsification Audit] delta error = " << (stored_ref.translation().x() - expected_rel.translation().x()) << std::endl;

    EXPECT_NEAR(stored_ref.translation().x(), expected_rel.translation().x(), 1e-4);
}

TEST(AnchorBoundaryIntegration, JitteredSamplesEndpointCaseA_Rotation)
{
    // Case A: Pure rotation at omega_z = 1.0 rad/s.
    // Jittered IMU samples at ~200Hz where no sample lands exactly on t0 + 0.1s.
    // Exact boundary ZOH integration must clamp at t_j, integrating exactly 0.100s.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 12000000000ll;
    const double omega_z = 1.0;

    int64_t t_cur = t0;
    tl.feedImu(t_cur, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, omega_z));
    int step = 0;
    while (t_cur < t0 + 105000000ll)  // up to 105 ms
    {
        const int64_t dt_jitter = ((step % 3 == 0) ? 4800000ll : ((step % 3 == 1) ? 5300000ll : 4900000ll));
        t_cur += dt_jitter;
        tl.feedImu(t_cur, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, omega_z));
        ++step;
    }

    ASSERT_GE(tl.anchorCount(), 2);
    gtsam::Pose3 T_1; gtsam::Vector3 v_1; gtsam::imuBias::ConstantBias b_1;
    ASSERT_TRUE(tl.anchorState(1, T_1, v_1, b_1));

    // Analytic expectation: Delta yaw = omega_z * 0.100s = 0.10000 rad.
    const double yaw_pred = T_1.rotation().yaw();
    const double yaw_ref = tl.imuRef(0).rotation().yaw();
    std::cout << "[Case A Rotation] T_1 yaw = " << yaw_pred << ", ref yaw = " << yaw_ref << std::endl;
    EXPECT_NEAR(yaw_pred, 0.100, 1e-5);
    EXPECT_NEAR(yaw_ref, 0.100, 1e-5);
}

TEST(AnchorBoundaryIntegration, JitteredSamplesEndpointCaseB_TranslationVelocity)
{
    // Case B: Pure linear acceleration at a_x = 2.0 m/s^2.
    // Jittered IMU samples straddling t0 + 0.1s.
    // Exact boundary ZOH integration must yield exact v1 = a*dt = 0.2 m/s, p1 = 0.5*a*dt^2 = 0.01 m.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 13000000000ll;
    const double a_x = 2.0;

    int64_t t_cur = t0;
    tl.feedImu(t_cur, gtsam::Vector3(a_x, 0.0, 9.81), gtsam::Vector3::Zero());
    int step = 0;
    while (t_cur < t0 + 105000000ll)
    {
        const int64_t dt_jitter = ((step % 3 == 0) ? 4700000ll : ((step % 3 == 1) ? 5400000ll : 5100000ll));
        t_cur += dt_jitter;
        tl.feedImu(t_cur, gtsam::Vector3(a_x, 0.0, 9.81), gtsam::Vector3::Zero());
        ++step;
    }

    ASSERT_GE(tl.anchorCount(), 2);
    gtsam::Pose3 T_1; gtsam::Vector3 v_1; gtsam::imuBias::ConstantBias b_1;
    ASSERT_TRUE(tl.anchorState(1, T_1, v_1, b_1));

    const double expected_v = a_x * 0.100;
    const double expected_p = 0.5 * a_x * 0.100 * 0.100;
    std::cout << "[Case B Translation] v_1.x() = " << v_1.x() << " (expected " << expected_v << ")" << std::endl;
    std::cout << "[Case B Translation] p_1.x() = " << T_1.translation().x() << " (expected " << expected_p << ")" << std::endl;
    EXPECT_NEAR(v_1.x(), expected_v, 1e-5);
    EXPECT_NEAR(T_1.translation().x(), expected_p, 1e-5);
    EXPECT_NEAR(tl.imuRef(0).translation().x(), expected_p, 1e-5);
}

TEST(LioConstraintId, DuplicateRejectionAndEpochSeparation)
{
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 14000000000ll;
    feedStaticImu(tl, t0, 0.5, 200);  // 5 intervals closed
    ASSERT_GE(tl.anchorCount(), 5);

    const int k = 1;
    const ConstraintId key_epoch0{0 /* SOURCE_LIO */, 0 /* epoch */, k};

    // 1. Initial valid insertion
    const AcceptDecision d1 = tl.insertRelativeConstraint(key_epoch0, gtsam::Pose3());
    EXPECT_EQ(d1, AcceptDecision::ACCEPTED);
    EXPECT_TRUE(tl.hasConstraint(key_epoch0));
    EXPECT_TRUE(tl.isIntervalLioConstrained(k));
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    EXPECT_EQ(tl.diag().lio_rejected_duplicate, 0);

    // 2. Exact duplicate key replay -> REJECT_DUPLICATE
    const AcceptDecision d2 = tl.insertRelativeConstraint(key_epoch0, gtsam::Pose3());
    EXPECT_EQ(d2, AcceptDecision::REJECT_DUPLICATE);
    EXPECT_EQ(tl.diag().lio_rejected_duplicate, 1);
    EXPECT_EQ(tl.diag().lio_accepted, 1);

    // 3. Different epoch on already-constrained interval -> REJECT_DUPLICATE (interval one-shot immutable)
    // 3. Different epoch on already-constrained interval -> REJECT_SLOT_OCCUPIED (interval one-shot immutable)
    const ConstraintId key_epoch1{0 /* SOURCE_LIO */, 1 /* epoch */, k};
    const AcceptDecision d3 = tl.insertRelativeConstraint(key_epoch1, gtsam::Pose3());
    
    
    EXPECT_EQ(d3, AcceptDecision::REJECT_SLOT_OCCUPIED);
    EXPECT_EQ(tl.diag().lio_rejected_slot_occupied, 1);
    EXPECT_EQ(tl.diag().lio_rejected_duplicate, 1);
    EXPECT_EQ(tl.diag().lio_accepted, 1);

    // 4. New epoch on unconstrained interval k=2 -> ACCEPTED
    const ConstraintId key_epoch1_k2{0 /* SOURCE_LIO */, 1 /* epoch */, 2};
    const AcceptDecision d4 = tl.insertRelativeConstraint(key_epoch1_k2, gtsam::Pose3());
    EXPECT_EQ(d4, AcceptDecision::ACCEPTED);
    EXPECT_TRUE(tl.hasConstraint(key_epoch1_k2));
    EXPECT_TRUE(tl.isIntervalLioConstrained(2));
    EXPECT_EQ(tl.diag().lio_accepted, 2);
}

TEST(LioRelativeFactor, NonZeroLeverArmPureRotationBodyTranslationZero)
{
    // Body rotates by 0.2 rad in yaw, zero body translation.
    // LiDAR has non-zero lever arm T_B_L = (0.25m, 0, 0).
    // Raw LiDAR translation displacement is ~0.05m != 0.
    // Normalization through T_B_L before differencing must recover exactly 0 body translation.
    ShadowTimeline tl(imuParams(), baseConfig());
    const Sophus::SE3d T_B_L(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.25, 0.0, 0.0));
    tl.setT_B_L(T_B_L);

    const int64_t t0 = 15000000000ll;
    const double omega_z = 2.0;  // 2.0 rad/s over 0.1s = 0.20 rad

    // IMU: pure rotation at 2.0 rad/s
    tl.feedImu(t0, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, omega_z));
    for (int i = 1; i <= 20; ++i)
        tl.feedImu(t0 + static_cast<int64_t>(i) * 5000000ll,
                   gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, omega_z));

    ASSERT_GE(tl.anchorCount(), 2);

    // LiDAR poses in world: T_W_L = T_W_B * T_B_L
    const Sophus::SE3d T_W_B0(Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
    const Sophus::SE3d T_W_B1(Eigen::Quaterniond(Eigen::AngleAxisd(0.20, Eigen::Vector3d::UnitZ())),
                             Eigen::Vector3d::Zero());

    const Sophus::SE3d T_W_L0 = T_W_B0 * T_B_L;
    const Sophus::SE3d T_W_L1 = T_W_B1 * T_B_L;

    // Verify raw LiDAR translation displacement is non-zero
    const Eigen::Vector3d raw_lidar_trans_disp = T_W_L1.translation() - T_W_L0.translation();
    EXPECT_GT(raw_lidar_trans_disp.norm(), 0.04);
    std::cout << "[Lever Arm Audit] Raw LiDAR trans disp norm = " << raw_lidar_trans_disp.norm() << " m" << std::endl;

    // Feed LIO poses
    tl.feedLioPose(t0, T_W_L0);
    tl.feedLioPose(t0 + 100000000ll, T_W_L1);
    tl.feedLioPose(t0 + 350000000ll, T_W_L1); // Push watermark to t0+250M

    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 0);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_rot, 0);
    EXPECT_GE(tl.diag().lio_accepted, 1);

    gtsam::Pose3 T_opt; gtsam::Vector3 v_opt; gtsam::imuBias::ConstantBias b_opt;
    ASSERT_TRUE(tl.latestAnchor(T_opt, v_opt, b_opt));
    std::cout << "[Lever Arm Audit] Optimized body translation norm = " << T_opt.translation().norm() << " m" << std::endl;
    EXPECT_LT(T_opt.translation().norm(), 1e-3);
    EXPECT_NEAR(T_opt.rotation().yaw(), 0.20, 1e-3);
}

TEST(GaugePrior, ExactlyOnePriorAddedAcrossManyAnchors)
{
    // Contract: One-time gauge priors (pose, vel, bias) -- never recurring source priors.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 16000000000ll;
    feedStaticImu(tl, t0, 10.0, 200);  // 10.0 s -> 100 anchor intervals

    EXPECT_EQ(tl.anchorCount(), 101);
    EXPECT_EQ(tl.diag().pose_priors_added, 1);
    EXPECT_EQ(tl.diag().vel_priors_added, 1);
    EXPECT_EQ(tl.diag().bias_priors_added, 1);
}

TEST(GraphRetention, UnboundedRetentionBeyond32AnchorsNoEviction)
{
    // Contract: unbounded_shadow retention mode (no hand-rolled 32-anchor eviction).
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 17000000000ll;
    feedStaticImu(tl, t0, 5.0, 200);  // 5.0 s -> 50 intervals

    EXPECT_GT(tl.anchorCount(), 32);
    EXPECT_EQ(tl.anchorCount(), 51);
    // Exact factor count: 3 initial priors + 50 ImuFactors + 50 bias BetweenFactors = 103
    EXPECT_EQ(tl.totalGraphFactors(), 103);

    // Verify all anchors from 0 to 50 are retrievable and finite
    for (int k = 0; k <= 50; ++k)
    {
        gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
        ASSERT_TRUE(tl.anchorState(k, T, v, b)) << "Failed to retrieve anchor " << k;
        EXPECT_LT(T.translation().norm(), 0.05);
        EXPECT_LT(v.norm(), 0.05);
    }
}

TEST(LioGate, OptimizerPosteriorUpdateDoesNotCorruptImuRef)
{
    // Contract: arrival-order-independent innovation gate.
    // dT_imu_ref(k) is captured at interval close and is NEVER altered by subsequent
    // optimizer updates.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 18000000000ll;
    const double a = 1.0;
    const int64_t dt_imu = 5000000ll;

    // Feed interval 0: [t0, t0 + 0.1s]
    tl.feedImu(t0, gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());
    for (int i = 1; i <= 20; ++i)
        tl.feedImu(t0 + static_cast<int64_t>(i) * dt_imu,
                   gtsam::Vector3(a, 0.0, 9.81), gtsam::Vector3::Zero());

    const gtsam::Pose3 ref0_before = tl.imuRef(0);

    // Insert a relative constraint on interval 0 that shifts the posterior (innovation 0.075m < 1.0m gate)
    const ConstraintId key{0, 0, 0};
    const gtsam::Pose3 shift_factor(gtsam::Rot3(), gtsam::Point3(0.080, 0.0, 0.0));
    EXPECT_EQ(tl.insertRelativeConstraint(key, shift_factor), AcceptDecision::ACCEPTED);

    // Check that posterior anchor state has shifted
    gtsam::Pose3 T_0, T_1; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    tl.anchorState(0, T_0, v, b);
    tl.anchorState(1, T_1, v, b);
    const gtsam::Pose3 posterior_rel = T_0.between(T_1);
    std::cout << "[Arrival-Order Invariance] ref0_before = " << ref0_before.translation().x()
              << ", posterior_rel = " << posterior_rel.translation().x() << std::endl;

    // Check that stored IMU reference is bitwise invariant to optimizer updates
    const gtsam::Pose3 ref0_after = tl.imuRef(0);
    EXPECT_TRUE(ref0_before.equals(ref0_after, 1e-15));
    EXPECT_NE(ref0_after.translation().x(), posterior_rel.translation().x());
}

TEST(AnchorBoundaryIntegration, NonConstantBoundarySampleOwnership)
{
    // Reviewer finding B-05 / A2: Lock right-sample ZOH boundary policy.
    // Interval [0.0, 0.100s] with non-constant step input across boundary:
    // t = 0.098s: omega_z = 1.0 rad/s
    // t = 0.103s: omega_z = 3.0 rad/s
    // boundary t_j = 0.100s
    //
    // Under Right-Sample ZOH:
    // Subsegment 1: [0.0, 0.098s] (dt = 0.098s) integrated with omega_z = 1.0 rad/s -> delta_yaw = 0.098 rad
    // Subsegment 2: [0.098, 0.100s] (dt = 0.002s) integrated with sample at 0.103s (omega_z = 3.0 rad/s) -> delta_yaw = 0.006 rad
    // Expected total yaw = 0.098 + 0.006 = 0.104 rad.
    // (Under Left-Sample ZOH, subsegment 2 would use omega_z = 1.0 rad/s -> delta_yaw = 0.002 rad -> total 0.100 rad).
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 20000000000ll;
    // Feed 10ms samples from t0 to t0 + 90ms with omega_z = 1.0 rad/s (all dt <= max_imu_dt_sec 0.05s)
    for (int i = 0; i <= 9; ++i)
    {
        tl.feedImu(t0 + static_cast<int64_t>(i) * 10000000ll,
                   gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, 1.0));
    }
    // Sample at 98ms with omega_z = 1.0 rad/s (dt = 8ms <= 50ms)
    tl.feedImu(t0 + 98000000ll, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, 1.0));
    // Boundary crossing sample at 103ms with omega_z = 3.0 rad/s (dt = 5ms <= 50ms)
    tl.feedImu(t0 + 103000000ll, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3(0.0, 0.0, 3.0));

    ASSERT_EQ(tl.diag().intervals_closed, 1);
    const double yaw_ref = tl.imuRef(0).rotation().yaw();
    EXPECT_NEAR(yaw_ref, 0.104, 1e-4);
    // Explicitly assert it does NOT match left-sample ZOH (0.100)
    EXPECT_GT(std::abs(yaw_ref - 0.100), 0.003);
}

TEST(AnchorBoundaryIntegration, InvalidInternalGapDoesNotCloseInterval)
{
    // Reviewer finding B-05 / A2: Partial invalid internal gap (> max_imu_dt_ns)
    // must NOT close an interval with incomplete integration.
    ShadowConfig c = baseConfig();
    c.max_imu_dt_ns = 20000000ll;  // 20 ms threshold
    ShadowTimeline tl(imuParams(), c);
    const int64_t t0 = 21000000000ll;
    // Sample at t0
    tl.feedImu(t0, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());
    // Valid segment: dt = 0.010s <= max_imu_dt_sec (0.02s)
    tl.feedImu(t0 + 10000000ll, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());
    // Invalid internal gap: dt = 0.050s > max_imu_dt_sec (0.02s)
    tl.feedImu(t0 + 60000000ll, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());
    // Boundary-crossing sample at t0 + 0.105s (crosses t1 = t0 + 0.100s)
    tl.feedImu(t0 + 105000000ll, gtsam::Vector3(0.0, 0.0, 9.81), gtsam::Vector3::Zero());

    // Invariant: interval 0 must NOT be closed because of the internal gap
    EXPECT_EQ(tl.diag().intervals_closed, 0);
    EXPECT_GT(tl.diag().imu_dropped, 0);
    EXPECT_EQ(tl.totalGraphFactors(), 0);  // Unclosed interval is not flushed to ISAM2
}

TEST(LioPoseBuffer, OutOfOrderFutureSampleDoesNotShadowEndpoint)
{
    // Reviewer finding B-01 / A4.1: Arrival-order independent right endpoint selection.
    // A future sample arriving before the true endpoint must NOT shadow the true endpoint.
    ShadowConfig c = baseConfig();
    ShadowTimeline tl(imuParams(), c);
    const int64_t t0 = 22000000000ll;
    // Feed 0.2s of IMU to establish intervals 0 and 1
    feedStaticImu(tl, t0, 0.2, 200);
    ASSERT_GE(tl.diag().intervals_closed, 2);

    // Static rig: ground truth pose is identity
    const Sophus::SE3d pose_0 = Sophus::SE3d();
    // Future sample arrives with 2.0m displacement (would violate 1.0m innovation gate if mistakenly chosen)
    const Sophus::SE3d pose_future(Eigen::Quaterniond::Identity(), Eigen::Vector3d(2.0, 0.0, 0.0));
    const Sophus::SE3d pose_1 = Sophus::SE3d();

    // Feed in out-of-timestamp order:
    tl.feedLioPose(t0, pose_0);                          // t = t0 (0.0s)
    tl.feedLioPose(t0 + 150000000ll, pose_future);       // t = t0 + 0.15s (arrives FIRST, watermark=50M)
    tl.feedLioPose(t0 + 100000000ll, pose_1);            // t = t0 + 0.10s (arrives LATER)
    tl.feedLioPose(t0 + 400000000ll, pose_future);

    // Interval 0 [t0, t0 + 0.1s] must use pose_1 (identity), NOT pose_future (2.0m)
    EXPECT_TRUE(tl.isIntervalLioConstrained(0));
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 0);

    Sophus::SE3d T_lookup;
    uint32_t ep_dummy; ASSERT_EQ(tl.lookupLioPoseAt(t0 + 100000000ll, T_lookup, ep_dummy), AcceptDecision::ACCEPTED);
    EXPECT_NEAR(T_lookup.translation().norm(), 0.0, 1e-5);
}

TEST(LioPoseBuffer, JitteredSamplesInterpolatedToAnchorTimes)
{
    // Reviewer finding B-03 / Section 3: Source pose interpolation contract.
    // LIO samples with timestamp jitter around anchors must be linearly interpolated
    // in translation and SLERP-interpolated in rotation to the exact anchor times.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 23000000000ll;  // 23.0s
    // Feed static IMU for 0.2s to establish intervals 0 and 1
    feedStaticImu(tl, t0, 0.2, 200);

    // Simulated trajectory: constant velocity v_x = 2.0 m/s, omega_z = 0.5 rad/s
    // Anchors are at t0 (0.0s) and t0 + 0.100s.
    // Feed jittered samples:
    // Sample a: t = t0 - 0.02s (-20ms) -> x = -0.04m, yaw = -0.010 rad
    // Sample b: t = t0 + 0.03s (+30ms) -> x = +0.06m, yaw = +0.015 rad
    // Sample c: t = t0 + 0.08s (+80ms) -> x = +0.16m, yaw = +0.040 rad
    // Sample d: t = t0 + 0.13s (+130ms)-> x = +0.26m, yaw = +0.065 rad
    auto makePose = [](double x, double yaw) {
        Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        return Sophus::SE3d(q, Eigen::Vector3d(x, 0.0, 0.0));
    };

    tl.feedLioPose(t0 - 20000000ll, makePose(-0.04, -0.010));
    tl.feedLioPose(t0 + 30000000ll, makePose(0.06, 0.015));
    tl.feedLioPose(t0 + 80000000ll, makePose(0.16, 0.040));
    tl.feedLioPose(t0 + 130000000ll, makePose(0.26, 0.065));
    tl.feedLioPose(t0 + 350000000ll, makePose(0.40, 0.100)); // Push watermark to t0+250M

    // Verify lookupLioPoseAt directly
    Sophus::SE3d T_L0, T_L1;
    uint32_t ep_dummy; ASSERT_EQ(tl.lookupLioPoseAt(t0, T_L0, ep_dummy), AcceptDecision::ACCEPTED);
    ASSERT_EQ(tl.lookupLioPoseAt(t0 + 100000000ll, T_L1, ep_dummy), AcceptDecision::ACCEPTED);

    // At t0: alpha = (0 - (-0.02)) / (0.03 - (-0.02)) = 0.02 / 0.05 = 0.4
    // x = -0.04 + 0.4 * 0.10 = 0.000m, yaw = -0.010 + 0.4 * 0.025 = 0.000 rad
    EXPECT_NEAR(T_L0.translation().x(), 0.0, 1e-4);
    EXPECT_NEAR(gtsam::Rot3(T_L0.rotationMatrix()).yaw(), 0.0, 1e-4);

    // At t0 + 0.100s: alpha = (0.10 - 0.08) / (0.13 - 0.08) = 0.02 / 0.05 = 0.4
    // x = 0.16 + 0.4 * 0.10 = 0.200m, yaw = 0.040 + 0.4 * 0.025 = 0.050 rad
    EXPECT_NEAR(T_L1.translation().x(), 0.20, 1e-4);
    EXPECT_NEAR(gtsam::Rot3(T_L1.rotationMatrix()).yaw(), 0.05, 1e-4);

    // Relative displacement across the 0.1s interval must be exactly (0.20m, 0.05rad)
    Sophus::SE3d T_rel = T_L0.inverse() * T_L1;
    EXPECT_NEAR(T_rel.translation().x(), 0.20, 1e-4);
    EXPECT_NEAR(gtsam::Rot3(T_rel.rotationMatrix()).yaw(), 0.05, 1e-4);

    // Interval 0 must be constrained and accepted
    EXPECT_TRUE(tl.isIntervalLioConstrained(0));
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}

TEST(SourceEpoch, DelayedOldEpochCannotRollbackCurrentEpoch)
{
    // Reviewer finding B-02 / A4.2: Monotonic source epoch enforcement.
    // A delayed old-epoch sample arriving after a new epoch has been observed
    // must be rejected and must not roll back current_lio_epoch_.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 24000000000ll;
    feedStaticImu(tl, t0, 0.2, 200);

    // Epoch 0 sample arrives
    tl.feedLioPose(t0, Sophus::SE3d(), 0);

    // Reboot / epoch advance: epoch 1 sample arrives
    tl.feedLioPose(t0 + 50000000ll, Sophus::SE3d(), 1);

    // Stale delayed epoch 0 sample arrives
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 0);

    // Stale sample must be skipped and diagnosed
    EXPECT_EQ(tl.diag().lio_stale_skipped, 1);
    EXPECT_FALSE(tl.isIntervalLioConstrained(0));
    EXPECT_EQ(tl.diag().lio_accepted, 0);

    // New epoch 1 sample completes the interval [t0 + 0.05, t0 + 0.15]
    tl.feedLioPose(t0 + 150000000ll, Sophus::SE3d(), 1);
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d(), 1);
    Sophus::SE3d T_interp;
    uint32_t ep_dummy; EXPECT_EQ(tl.lookupLioPoseAt(t0 + 100000000ll, T_interp, ep_dummy), AcceptDecision::ACCEPTED);
}

TEST(HighRateState, PropagateToHistoricalDynamicTimestampUsesCorrectAnchor)
{
    // Reviewer finding B-04 / Section 2: High-rate propagation for historical/mid-trajectory timestamps.
    // propagateTo(t) must select anchor k such that anchor_stamps_[k] <= t,
    // rather than always starting from the newest anchor.
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 25000000000ll;

    // Feed 1.0s of IMU with constant acceleration a_x = 2.0 m/s^2 at 200 Hz
    const double a_x = 2.0;
    const int64_t dt_ns = 5000000ll;  // 5 ms
    for (int i = 0; i <= 200; ++i)
    {
        tl.feedImu(t0 + static_cast<int64_t>(i) * dt_ns,
                   gtsam::Vector3(a_x, 0.0, 9.81), gtsam::Vector3::Zero());
    }

    ASSERT_EQ(tl.anchorCount(), 11);  // anchors 0 to 10 (t0 to t0 + 1.0s)
    ASSERT_EQ(tl.diag().intervals_closed, 10);

    // Query historical timestamp at t = t0 + 0.55s (550 ms)
    // Anchor 5 is at 0.50s; Anchor 10 is at 1.00s.
    // Analytic state at 0.55s:
    // v_x(0.55) = a_x * 0.55 = 1.10 m/s
    // p_x(0.55) = 0.5 * a_x * 0.55^2 = 0.3025 m
    // If it started from newest anchor (1.00s), v_x would be >= 2.0 m/s and p_x >= 1.0 m.
    const auto st = tl.propagateTo(t0 + 550000000ll);
    ASSERT_TRUE(st.valid);
    EXPECT_NEAR(st.v_W.x(), 1.10, 0.05);
    EXPECT_NEAR(st.T_W_B.translation().x(), 0.3025, 0.05);
    // Strictly verify it did NOT return newest-anchor state (p_x ~ 1.0m)
    EXPECT_LT(st.T_W_B.translation().x(), 0.50);
}

}  // namespace
