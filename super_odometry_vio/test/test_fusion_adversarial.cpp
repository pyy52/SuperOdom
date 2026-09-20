// Phase 4B-1 R3 INDEPENDENT ADVERSARIAL FALSIFICATION SUITE
// ---------------------------------------------------------------------------
// Author role: adversarial validation agent (Agent B), NOT the implementer.
// Target revision: de6a3b0c63293442aca85ed3170f54e86f0e5104 (test-only branch
//                  audit/r3-falsification; production sources untouched).
//
// Every test here is derived from the *signed* contract only:
//   docs/FUSION_TIMELINE_DESIGN.md (Phase 4A decisions §4/§5/§6/§7) and the
//   frozen header comment of
//   super_odometry_vio/include/super_odometry_vio/fusion_2021/shadow_timeline.hpp
// (exact measurement-time grid; immutable pre-fusion dT_imu_ref; body-frame
//  differencing through T_B_L; right-sample ZOH; one-shot (source, epoch, k)
//  identity; monotonic source epoch; no silent dt fallback).
//
// The R3 implementation agent's prose is deliberately NOT used as evidence.
//
// F-sections map 1:1 onto the audit tasking (F1..F12). Tests marked
// CHARACTERISATION pin down the *observed* behaviour of a contract corner the
// signed design does not define instead of asserting an invented policy; they
// are reported as contract ambiguities in the review document.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>

#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"


namespace
{

using namespace super_odometry_vio::fusion_2021;

// ---------------------------------------------------------------------------
// helpers (independently re-derived, not copied from the R3 suite)
// ---------------------------------------------------------------------------

constexpr double kGravity = 9.81;

ShadowConfig defaultCfg()
{
    ShadowConfig c;            // 10 Hz grid, 0.5 s lateness, 0.05 s IMU dt,
    c.anchor_rate_hz = 10.0;   // 0.15 s interpolation gap, 1.0 m / 0.35 rad gate
    return c;                  // -- the frozen initial values.
}

std::shared_ptr<gtsam::PreintegrationParams> imuParams()
{
    auto p = gtsam::PreintegrationParams::MakeSharedU(kGravity);
    p->accelerometerCovariance = gtsam::I_3x3 * 1e-6;
    p->gyroscopeCovariance = gtsam::I_3x3 * 1e-6;
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;
    return p;
}

Sophus::SE3d se3YawT(double yaw, double x, double y, double z)
{
    return Sophus::SE3d(
        Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())),
        Eigen::Vector3d(x, y, z));
}

gtsam::Pose3 toGtsam(const Sophus::SE3d& T)
{
    return gtsam::Pose3(gtsam::Rot3(T.rotationMatrix()),
                        gtsam::Point3(T.translation()));
}

double rotErr(const gtsam::Pose3& a, const gtsam::Pose3& b)
{
    return gtsam::Pose3::Logmap(a.between(b)).head<3>().norm();
}

double transErr(const gtsam::Pose3& a, const gtsam::Pose3& b)
{
    return gtsam::Pose3::Logmap(a.between(b)).tail<3>().norm();
}

struct ImuRec
{
    int64_t t;
    gtsam::Vector3 acc;
    gtsam::Vector3 gyro;
};

// feed a uniform IMU train [t_start, t_start + n*dt]
void feedImuTrain(ShadowTimeline& tl, int64_t t_start, int n, int64_t dt_ns,
                  const gtsam::Vector3& acc, const gtsam::Vector3& gyro,
                  std::vector<ImuRec>* rec = nullptr)
{
    for (int i = 0; i <= n; ++i)
    {
        const int64_t t = t_start + static_cast<int64_t>(i) * dt_ns;
        tl.feedImu(t, acc, gyro);
        if (rec) rec->push_back({t, acc, gyro});
    }
}

void feedRestImu(ShadowTimeline& tl, int64_t t_start, double seconds, int rate_hz,
                 std::vector<ImuRec>* rec = nullptr)
{
    const int64_t dt = static_cast<int64_t>(1e9 / rate_hz);
    const int n = static_cast<int>(seconds * rate_hz);
    feedImuTrain(tl, t_start, n, dt, gtsam::Vector3(0.0, 0.0, kGravity),
                 gtsam::Vector3::Zero(), rec);
}


namespace
{

// ---------------------------------------------------------------------------
// independent right-sample ZOH oracle
// ---------------------------------------------------------------------------
// Signed policy (design §6 + A2): the sub-interval (t_prev, t_cur] is
// integrated with the measurement whose sample time IS t_cur (right endpoint
// ownership); the terminal sub-interval (t_prev, t_j] with t_j not a sample
// time is integrated with the first sample at or after t_j.  Re-derived from
// the documented policy, not from the implementation body.
struct ZohOracle
{
    bool complete{false};
    double covered_sec{0.0};
    gtsam::NavState state;  // predicted from an identity base
};

ZohOracle zohOracle(const std::vector<ImuRec>& recs, int64_t t_i, int64_t t_j,
                    double max_dt, bool left_sample_owner = false)
{
    gtsam::PreintegratedImuMeasurements pim(imuParams(),
                                            gtsam::imuBias::ConstantBias());
    int64_t covered = t_i;
    bool gap = false;
    ImuRec prev{t_i, gtsam::Vector3::Zero(), gtsam::Vector3::Zero()};
    for (const auto& s : recs)
    {
        if (s.t <= t_i) { prev = s; continue; }
        const bool terminal = (s.t >= t_j);
        const int64_t seg_end = terminal ? t_j : s.t;
        if (seg_end <= covered)
        {
            if (terminal) break;
            prev = s;
            continue;
        }
        const double dt = static_cast<double>(seg_end - covered) * 1e-9;
        const gtsam::Vector3 acc = left_sample_owner ? prev.acc : s.acc;
        const gtsam::Vector3 gyro = left_sample_owner ? prev.gyro : s.gyro;
        if (dt <= 0.0 || dt > max_dt)
        {
            gap = true;
            covered = seg_end;
            if (terminal) break;
            prev = s;
            continue;
        }
        pim.integrateMeasurement(acc, gyro, dt);
        covered = seg_end;
        if (terminal) break;
        prev = s;
    }
    ZohOracle o;
    o.complete = (!gap && covered == t_j);
    o.covered_sec = static_cast<double>(covered - t_i) * 1e-9;
    const gtsam::NavState base(gtsam::Pose3(), gtsam::Vector3::Zero());
    o.state = pim.predict(base, gtsam::imuBias::ConstantBias());
    return o;
}

// pure coverage bookkeeping (used to prove the exact integrated-time budget)
double coveredSec(const std::vector<ImuRec>& recs, int64_t t_i, int64_t t_j)
{
    int64_t covered = t_i;
    for (const auto& s : recs)
    {
        if (s.t <= t_i) continue;
        const int64_t seg_end = (s.t >= t_j) ? t_j : s.t;
        if (seg_end <= covered) { if (s.t >= t_j) break; continue; }
        covered = seg_end;
        if (s.t >= t_j) break;
    }
    return static_cast<double>(covered - t_i) * 1e-9;
}

}  // namespace


// ===========================================================================
// F1 - LIO lookup must be arrival-order independent
// ===========================================================================
namespace
{

// F1 rig: constant body velocity + constant yaw rate, LiDAR lever arm.
const Sophus::SE3d kF1TBL = se3YawT(0.20, 0.12, 0.03, 0.05);

Sophus::SE3d f1TWL(double t)
{
    const Sophus::SE3d T_W_B = se3YawT(0.8 * t, 0.30 + 1.2 * t, -0.4 * t, 0.05 * t);
    return T_W_B * kF1TBL;
}

struct F1Entry
{
    int64_t stamp;
    Sophus::SE3d T_W_L;
};

std::vector<std::vector<int>> f1Orders()
{
    return {
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13},   // sorted
        {13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0},   // reverse
        {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 0, 1},   // exact-anchor samples last
        {0, 1, 2, 3, 5, 6, 8, 10, 12, 4, 7, 9, 11, 13},   // right brackets last
        {6, 11, 3, 13, 1, 8, 4, 0, 9, 2, 12, 7, 5, 10}    // fixed permutation
    };
}

struct F1Result
{
    std::vector<Sophus::SE3d> lookup;
    std::vector<gtsam::Pose3> anchor;
    std::vector<int> constrained;
    int accepted{0};
    int rej_trans{0};
    int rej_rot{0};
    size_t factors{0};
    int anchor_count{0};
};

F1Result runF1Order(int order_index, bool lio_before_imu)
{
    const int64_t t0 = 30000000000ll;
    const std::vector<int64_t> off_ms = {-30, 0,  20,  80,  105, 145, 195,
                                         215, 285, 320, 355, 405, 480, 520};
    std::vector<F1Entry> entries;
    for (size_t i = 0; i < off_ms.size(); ++i)
    {
        const int64_t stamp = t0 + off_ms[i] * 1000000ll;
        Sophus::SE3d T = f1TWL(static_cast<double>(off_ms[i]) * 1e-3);
        if (off_ms[i] == 0)
            T.translation() += Eigen::Vector3d(0.25, 0.0, 0.0);  // exact anchor sample
        if (off_ms[i] == 480)
            T.translation() += Eigen::Vector3d(2.0, 0.0, 0.0);   // gate violation
        entries.push_back({stamp, T});
    }

    ShadowTimeline tl(imuParams(), defaultCfg());
    tl.setT_B_L(kF1TBL);
    std::vector<ImuRec> recs;
    const auto order = f1Orders()[order_index];
    auto feed_lio = [&]() {
        for (int idx : order) tl.feedLioPose(entries[idx].stamp, entries[idx].T_W_L);
    };
    if (lio_before_imu)
    {
        feed_lio();
        feedRestImu(tl, t0, 0.5, 200, &recs);
    }
    else
    {
        feedRestImu(tl, t0, 0.5, 200, &recs);
        feed_lio();
    }

    F1Result r;
    r.accepted = tl.diag().lio_accepted;
    r.rej_trans = tl.diag().lio_rejected_innovation_trans;
    r.rej_rot = tl.diag().lio_rejected_innovation_rot;
    r.factors = tl.totalGraphFactors();
    r.anchor_count = tl.anchorCount();
    for (int k = 0; k <= 5; ++k)
    {
        Sophus::SE3d T;
        EXPECT_TRUE(tl.lookupLioPoseAt(0, t0 + static_cast<int64_t>(k) * 100000000ll, T));
        r.lookup.push_back(T);
        gtsam::Pose3 P;
        gtsam::Vector3 v;
        gtsam::imuBias::ConstantBias b;
        EXPECT_TRUE(tl.anchorState(k, P, v, b));
        r.anchor.push_back(P);
    }
    for (int k = 0; k <= 4; ++k) r.constrained.push_back(tl.isIntervalLioConstrained(k));
    return r;
}

}  // namespace

TEST(F1_ArrivalOrder, InterpolationGateAndPosteriorIdenticalInAllOrders)
{
    const F1Result ref = runF1Order(0, false);

    // the timestamp set must actually exercise both gate branches.
    // NB: isIntervalLioConstrained() means "this interval received its one and
    // only LIO decision" -- a terminal gate rejection also pins it (one-shot).
    ASSERT_EQ(ref.accepted, 4);
    ASSERT_EQ(ref.rej_trans, 1);
    ASSERT_EQ(ref.rej_rot, 0);
    ASSERT_EQ(ref.constrained[4], 1);

    for (int o = 0; o < 5; ++o)
    {
        for (int lio_first = 0; lio_first <= 1; ++lio_first)
        {
            const F1Result r = runF1Order(o, lio_first != 0);
            const std::string tag =
                "order=" + std::to_string(o) + " lio_before_imu=" + std::to_string(lio_first);
            EXPECT_EQ(r.accepted, ref.accepted) << tag;
            EXPECT_EQ(r.rej_trans, ref.rej_trans) << tag;
            EXPECT_EQ(r.rej_rot, ref.rej_rot) << tag;
            EXPECT_EQ(r.constrained, ref.constrained) << tag;
            EXPECT_EQ(r.factors, ref.factors) << tag << " graph factor count";
            EXPECT_EQ(r.anchor_count, ref.anchor_count) << tag << " anchor count";
            for (size_t k = 0; k < ref.lookup.size(); ++k)
                EXPECT_TRUE(r.lookup[k].matrix().isApprox(ref.lookup[k].matrix(), 1e-12))
                    << tag << " interpolated T_W_L at anchor " << k;
            for (size_t k = 0; k < ref.anchor.size(); ++k)
            {
                if (!r.anchor[k].equals(ref.anchor[k], 1e-9))
                {
                    const double dtrans =
                        (r.anchor[k].translation() - ref.anchor[k].translation()).norm();
                    const double drot =
                        (gtsam::Pose3::Logmap(r.anchor[k]).head<3>() -
                         gtsam::Pose3::Logmap(ref.anchor[k]).head<3>())
                            .norm();
                    std::cout << "[F1] DIVERGENT " << tag << " anchor " << k
                              << " dtrans=" << dtrans << " drot=" << drot << std::endl;
                }
                EXPECT_TRUE(r.anchor[k].equals(ref.anchor[k], 1e-9))
                    << tag << " posterior anchor " << k;
            }
        }
    }
}

TEST(F1_ExactAnchorSample, DominatesInterpolationInEveryOrder)
{
    // Exact sample at t_0 is displaced by +0.25 m from the analytic value, so
    // accepting the exact sample and interpolating give visibly different
    // answers -> the check has discriminating power.
    const Sophus::SE3d exact_shifted =
        [&] { Sophus::SE3d T = f1TWL(0.0); T.translation() += Eigen::Vector3d(0.25, 0, 0); return T; }();

    for (int o = 0; o < 5; ++o)
    {
        const F1Result r = runF1Order(o, false);
        EXPECT_TRUE(r.lookup[0].matrix().isApprox(exact_shifted.matrix(), 1e-12))
            << "order " << o;
    }
    const Eigen::Vector3d p_before = f1TWL(-0.03).translation();
    const Eigen::Vector3d p_after = f1TWL(0.02).translation();
    const Eigen::Vector3d interp = 0.4 * p_before + 0.6 * p_after;  // alpha = 0.6
    EXPECT_GT(std::abs(interp.x() - exact_shifted.translation().x()), 0.2);
}


// ===========================================================================
// F2 - duplicate timestamps and ambiguous bracketing
// ===========================================================================
TEST(F2_DuplicateTimestamps, ArrivalOrderSensitivity_DocumentedPolicyGap)
{
    // CHARACTERISATION. The signed design defines duplicate/out-of-order
    // rejection for the *IMU* interval buffer (design §6). It does NOT define a
    // duplicate policy for the *source* pose buffer. Two same-epoch LIO samples
    // with the identical stamp and different poses are therefore undefined
    // input; this test pins the observed behaviour so it cannot change silently.
    const int64_t t0 = 31000000000ll;
    const Sophus::SE3d A = se3YawT(0.0, 0.10, 0.0, 0.0);
    const Sophus::SE3d B = se3YawT(0.0, 0.90, 0.0, 0.0);
    const Sophus::SE3d C = se3YawT(0.0, 0.20, 0.0, 0.0);

    auto run = [&](bool a_first) {
        ShadowTimeline tl(imuParams(), defaultCfg());
        feedRestImu(tl, t0, 0.2, 200);
        tl.feedLioPose(t0 + 200000000ll, C);
        if (a_first) { tl.feedLioPose(t0, A); tl.feedLioPose(t0, B); }
        else         { tl.feedLioPose(t0, B); tl.feedLioPose(t0, A); }
        Sophus::SE3d out;
        EXPECT_TRUE(tl.lookupLioPoseAt(0, t0, out));
        return out;
    };

    const Sophus::SE3d a_first = run(true);
    const Sophus::SE3d b_first = run(false);
    std::cout << "[F2] duplicate-stamp anchor lookup: A-first x = "
              << a_first.translation().x() << " , B-first x = "
              << b_first.translation().x() << std::endl;

    // Observed: the first-arrived sample at that stamp wins (deque scan order).
    EXPECT_NEAR(a_first.translation().x(), A.translation().x(), 1e-12);
    EXPECT_NEAR(b_first.translation().x(), B.translation().x(), 1e-12);
    // => the same timestamp-identical input set yields two different anchor
    // poses depending on arrival order: silent arrival-order nondeterminism.
    EXPECT_GT(std::abs(a_first.translation().x() - b_first.translation().x()), 0.5);
}

TEST(F2_Bracketing, ClosestTimestampPairIsSelected_OrderIndependent)
{
    ShadowTimeline tl(imuParams(), defaultCfg());
    const int64_t t0 = 32000000000ll;
    feedRestImu(tl, t0, 0.3, 200);

    // Non-linear source motion p(t) = 0.5*a*t^2, a = 4 m/s^2, so that the
    // *choice* of bracket is numerically observable.
    auto p = [](double t) { return 0.5 * 4.0 * t * t; };
    auto feed_sample = [&](double t_s) {
        tl.feedLioPose(t0 + static_cast<int64_t>(std::llround(t_s * 1e9)),
                       se3YawT(0.0, p(t_s), 0.0, 0.0));
    };
    // reverse arrival order, two samples on each side of the anchor at 0.1 s
    feed_sample(0.17);
    feed_sample(0.12);
    feed_sample(0.09);
    feed_sample(0.04);

    Sophus::SE3d T;
    ASSERT_TRUE(tl.lookupLioPoseAt(0, t0 + 100000000ll, T));

    const double a_near = (0.10 - 0.09) / (0.12 - 0.09);
    const double near_interp = (1.0 - a_near) * p(0.09) + a_near * p(0.12);
    const double a_far = (0.10 - 0.04) / (0.17 - 0.04);
    const double far_interp = (1.0 - a_far) * p(0.04) + a_far * p(0.17);

    EXPECT_NEAR(T.translation().x(), near_interp, 1e-9);   // closest pair wins
    EXPECT_GT(std::abs(far_interp - near_interp), 1e-3);   // discriminating power
}


// ===========================================================================
// F3 - epoch monotonicity and stale-buffer isolation
// ===========================================================================
TEST(F3_Epoch, MonotonicEpochStaleIsolationAndNoCentralReset)
{
    ShadowTimeline tl(imuParams(), defaultCfg());
    const int64_t t0 = 33000000000ll;
    const Sophus::SE3d T_W_L;   // identity source pose

    // epoch 0 normal operation: anchors 0..3, intervals 0..2 closed
    feedRestImu(tl, t0, 0.3, 200);
    for (int k = 0; k <= 3; ++k)
        tl.feedLioPose(t0 + static_cast<int64_t>(k) * 100000000ll, T_W_L, 0);
    EXPECT_EQ(tl.diag().lio_accepted, 3);
    for (int k = 0; k <= 2; ++k) EXPECT_TRUE(tl.isIntervalLioConstrained(k));
    const size_t factors_epoch0 = tl.totalGraphFactors();

    // epoch jumps to 1. All future epoch-1 samples are buffered up front so
    // that each interval gets its one-shot decision AT CLOSE TIME with the
    // same-epoch brackets available (one-shot pinning: samples fed after the
    // close can no longer constrain it).
    tl.feedLioPose(t0 + 400000000ll, T_W_L, 1);
    tl.feedLioPose(t0 + 500000000ll, T_W_L, 1);
    tl.feedLioPose(t0 + 600000000ll, T_W_L, 1);
    tl.feedLioPose(t0 + 700000000ll, T_W_L, 1);
    EXPECT_EQ(tl.diag().lio_stale_skipped, 0);

    // a late epoch-0 sample arrives afterwards: dropped, never buffered
    tl.feedLioPose(t0 + 500000000ll, T_W_L, 0);
    EXPECT_EQ(tl.diag().lio_stale_skipped, 1);
    Sophus::SE3d stale_lookup;
    EXPECT_FALSE(tl.lookupLioPoseAt(0, t0 + 500000000ll, stale_lookup))
        << "stale-epoch sample must not be buffered";

    // epoch 1 continues (anchors 4..7, intervals 3..6 closed)
    feedRestImu(tl, t0 + 300000000ll, 0.4, 200);
    std::cout << "[F3] after epoch-1 IMU: closed=" << tl.diag().intervals_closed
              << " anchors=" << tl.anchorCount()
              << " acc=" << tl.diag().lio_accepted
              << " no_bracket=" << tl.diag().lio_no_bracket
              << " too_late=" << tl.diag().lio_rejected_too_late
              << " dup=" << tl.diag().lio_rejected_duplicate
              << " rej_t=" << tl.diag().lio_rejected_innovation_trans
              << " rej_r=" << tl.diag().lio_rejected_innovation_rot << std::endl;

    // interval 3 [0.3,0.4] could only be bracketed by a cross-epoch pair
    // (epoch-0 sample at 0.3 / epoch-1 sample at 0.4) -> must stay unconstrained
    EXPECT_FALSE(tl.isIntervalLioConstrained(3))
        << "old-epoch sample must not bracket a new-epoch anchor";
    EXPECT_TRUE(tl.isIntervalLioConstrained(4));
    EXPECT_TRUE(tl.isIntervalLioConstrained(5));

    // central graph lifecycle: a source epoch change never resets the central graph
    EXPECT_GE(tl.totalGraphFactors(), factors_epoch0);
    EXPECT_EQ(tl.diag().pose_priors_added, 1);
    EXPECT_EQ(tl.diag().vel_priors_added, 1);
    EXPECT_EQ(tl.diag().bias_priors_added, 1);
    EXPECT_EQ(tl.anchorCount(), 8);

    // old-epoch replay after the fact
    const int accepted_before = tl.diag().lio_accepted;
    tl.feedLioPose(t0 + 800000000ll, T_W_L, 0);
    EXPECT_EQ(tl.diag().lio_stale_skipped, 2);
    EXPECT_EQ(tl.diag().lio_accepted, accepted_before);
}


TEST(F3_Epoch, MultiStepJumpAndLateEpochsCannotRollBack)
{
    ShadowTimeline tl(imuParams(), defaultCfg());
    const int64_t t0 = 34000000000ll;
    feedRestImu(tl, t0, 0.5, 200);           // anchors 0..5, intervals 0..4 closed
    const Sophus::SE3d T_W_L;

    tl.feedLioPose(t0, T_W_L, 1);                        // epoch 0 -> 1
    tl.feedLioPose(t0 + 100000000ll, T_W_L, 3);          // 1 -> 3 jump
    tl.feedLioPose(t0 + 200000000ll, T_W_L, 2);          // late epoch 2
    tl.feedLioPose(t0 + 300000000ll, T_W_L, 1);          // late epoch 1
    tl.feedLioPose(t0 + 400000000ll, T_W_L, 3);          // epoch 3 continues
    tl.feedLioPose(t0 + 450000000ll, T_W_L, 3);
    tl.feedLioPose(t0 + 550000000ll, T_W_L, 3);

    EXPECT_EQ(tl.diag().lio_stale_skipped, 2);

    // current epoch never rolls backward: only epoch-3 samples are usable
    Sophus::SE3d T;
    EXPECT_TRUE(tl.lookupLioPoseAt(3, t0 + 100000000ll, T));
    EXPECT_TRUE(tl.lookupLioPoseAt(3, t0 + 400000000ll, T));
    EXPECT_FALSE(tl.lookupLioPoseAt(2, t0 + 200000000ll, T))
        << "late epoch-2 sample must not be buffered";
    EXPECT_FALSE(tl.lookupLioPoseAt(1, t0 + 300000000ll, T))
        << "late epoch-1 sample must not be buffered";

    // interval 4 [0.4,0.5] is bracketed by epoch-3 samples -> constrained;
    // interval 3 [0.3,0.4] has no epoch-3 left bracket -> cross-epoch isolated
    EXPECT_TRUE(tl.isIntervalLioConstrained(4));
    EXPECT_FALSE(tl.isIntervalLioConstrained(3));
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}


// ===========================================================================
// F4 - exact anchor-time interpolation, verified through the public gate
// ===========================================================================
namespace
{

// The gate compares the interval measurement T_Bi_Bj against the immutable
// IMU-only reference dT_imu_ref(k) with separate translation/rotation bounds.
// Bisecting those public bounds therefore *measures* |Logmap| of the
// implementation's own measurement without touching private state, and it is
// independent of any optimizer state (the reference is pre-fusion).
struct ProbeResult
{
    bool accepted{false};
    int rej_trans{0};
    int rej_rot{0};
};

template <typename F>
ProbeResult runProbe(F scenario, double trans_thr, double rot_thr)
{
    ShadowConfig c = defaultCfg();
    c.innovation_trans_m = trans_thr;
    c.innovation_rot_rad = rot_thr;
    ShadowTimeline tl(imuParams(), c);
    scenario(tl);
    std::cout << "[probe] thr=(" << trans_thr << "," << rot_thr << ")"
              << " closed=" << tl.diag().intervals_closed
              << " acc=" << tl.diag().lio_accepted
              << " no_bracket=" << tl.diag().lio_no_bracket
              << " rej_trans=" << tl.diag().lio_rejected_innovation_trans
              << " rej_rot=" << tl.diag().lio_rejected_innovation_rot
              << " too_late=" << tl.diag().lio_rejected_too_late << std::endl;
    ProbeResult r;
    r.accepted = (tl.diag().lio_accepted == 1);
    r.rej_trans = tl.diag().lio_rejected_innovation_trans;
    r.rej_rot = tl.diag().lio_rejected_innovation_rot;
    return r;
}

// Measured error = the smallest gate bound that still accepts (the predicate
// "accepted" is monotone: true iff trans/rot error <= threshold). Bisection on
// [0, hi] with: accepted -> the boundary lies in [lo, mid]; rejected -> in
// (mid, hi]. Returns the converged boundary (precision ~8*2^-45).
template <typename F>
double measureError(F scenario, bool rotation, double hi = 8.0)
{
    double lo = 0.0;
    for (int i = 0; i < 45; ++i)
    {
        const double mid = 0.5 * (lo + hi);
        const ProbeResult r = rotation ? runProbe(scenario, 1e9, mid)
                                       : runProbe(scenario, mid, 1e9);
        if (r.accepted) hi = mid;
        else lo = mid;
    }
    return hi;
}

// analytic T_W_L(t) for the F4 rig: constant body velocity + constant yaw rate
struct F4Rig
{
    double yaw_rate{0.0};
    Eigen::Vector3d v{0.0, 0.0, 0.0};
    Sophus::SE3d T_B_L;

    Sophus::SE3d T_W_L(double t) const
    {
        const Sophus::SE3d T_W_B = se3YawT(yaw_rate * t, v.x() * t, v.y() * t, v.z() * t);
        return T_W_B * T_B_L;
    }
    // analytic body-relative measurement over [t_i, t_j]
    gtsam::Pose3 measurement(double t_i, double t_j) const
    {
        const Sophus::SE3d T_l = T_B_L * T_W_L(t_i).inverse() * T_W_L(t_j) * T_B_L.inverse();
        return toGtsam(T_l);
    }
    // jittered sample set around the two anchors the scenario uses
    void feed(ShadowTimeline& tl, int64_t t0) const
    {
        const double off[4] = {-0.023, 0.037, 0.077, 0.123};
        for (double o : off)
            tl.feedLioPose(t0 + static_cast<int64_t>(std::llround(o * 1e9)), T_W_L(o), 0);
    }
};

// one-decision scenario: interval 0 only, anchors 0 and 0.1 s, static IMU.
// The LIO samples are buffered BEFORE the IMU train: interval 0 receives its
// one-shot decision at close time (0.1 s), so the bracket pair must already be
// in the buffer when the closing IMU sample arrives.
template <typename F>
void oneIntervalScenario(ShadowTimeline& tl, int64_t t0, F&& feed_lio)
{
    feed_lio(tl, t0);
    feedRestImu(tl, t0, 0.2, 200);
}

}  // namespace

TEST(F4_Interpolation, ConstantVelocityTranslationExactAtAnchorTimes)
{
    const int64_t t0 = 41000000000ll;
    F4Rig rig;
    rig.yaw_rate = 0.0;
    rig.v = Eigen::Vector3d(1.1, 0.0, 0.0);
    rig.T_B_L = se3YawT(0.0, 0.12, -0.03, 0.05);   // pure-translation lever arm

    ShadowTimeline tl(imuParams(), defaultCfg());
    tl.setT_B_L(rig.T_B_L);
    oneIntervalScenario(tl, t0, [&](ShadowTimeline& t, int64_t base) { rig.feed(t, base); });

    // interpolated source pose at the exact anchor times
    Sophus::SE3d T_i, T_j;
    ASSERT_TRUE(tl.lookupLioPoseAt(0, t0, T_i));
    ASSERT_TRUE(tl.lookupLioPoseAt(0, t0 + 100000000ll, T_j));
    std::cout << "[F4] direct-run diag: closed=" << tl.diag().intervals_closed
              << " anchors=" << tl.anchorCount()
              << " acc=" << tl.diag().lio_accepted
              << " no_bracket=" << tl.diag().lio_no_bracket
              << " rej_t=" << tl.diag().lio_rejected_innovation_trans
              << " rej_r=" << tl.diag().lio_rejected_innovation_rot
              << " too_late=" << tl.diag().lio_rejected_too_late << std::endl;
    EXPECT_TRUE(T_i.matrix().isApprox(rig.T_W_L(0.0).matrix(), 1e-9));
    EXPECT_TRUE(T_j.matrix().isApprox(rig.T_W_L(0.1).matrix(), 1e-9));

    // the implementation's own measurement, measured through the gate
    const auto scenario = [&](ShadowTimeline& t) {
        t.setT_B_L(rig.T_B_L);
        oneIntervalScenario(t, t0, [&](ShadowTimeline& x, int64_t b) { rig.feed(x, b); });
    };
    const double measured = measureError(scenario, false);
    const double expected = rig.measurement(0.0, 0.1).translation().norm();
    std::cout << "[F4] constant-velocity trans err: measured " << measured
              << " expected " << expected << std::endl;
    EXPECT_NEAR(measured, expected, 1e-6);

    // discriminating power: using the raw bracket samples (no interpolation to
    // the anchor times) would give the displacement over 0.046 s instead.
    const double raw_bracket = 1.1 * 0.046;
    EXPECT_GT(std::abs(expected - raw_bracket), 0.05);
}

TEST(F4_Interpolation, ConstantAngularVelocityExactAtEachAnchor)
{
    const int64_t t0 = 42000000000ll;
    F4Rig rig;
    rig.yaw_rate = 0.7;
    rig.v = Eigen::Vector3d(0.0, 0.0, 0.0);
    rig.T_B_L = se3YawT(0.0, 0.0, 0.0, 0.0);

    ShadowTimeline tl(imuParams(), defaultCfg());
    tl.setT_B_L(rig.T_B_L);
    oneIntervalScenario(tl, t0, [&](ShadowTimeline& t, int64_t base) { rig.feed(t, base); });

    Sophus::SE3d T_i, T_j;
    ASSERT_TRUE(tl.lookupLioPoseAt(0, t0, T_i));
    ASSERT_TRUE(tl.lookupLioPoseAt(0, t0 + 100000000ll, T_j));
    // verify interception at BOTH anchors, not just the total relative angle
    EXPECT_NEAR(gtsam::Rot3(T_i.rotationMatrix()).yaw(), 0.0, 1e-9);
    EXPECT_NEAR(gtsam::Rot3(T_j.rotationMatrix()).yaw(), 0.07, 1e-9);

    const auto scenario = [&](ShadowTimeline& t) {
        t.setT_B_L(rig.T_B_L);
        oneIntervalScenario(t, t0, [&](ShadowTimeline& x, int64_t b) { rig.feed(x, b); });
    };
    const double measured = measureError(scenario, true);
    std::cout << "[F4] constant-yaw-rate rot err: measured " << measured << std::endl;
    EXPECT_NEAR(measured, 0.07, 1e-6);
}


TEST(F4_SE3Order, InterpolateToAnchorThenLeverArmThenDifference)
{
    // Combined SE(3): body translation + rotation + non-zero lever arm.
    // The static IMU makes the frozen reference exactly identity, so the gate
    // probe measures |Logmap| of the implementation's own T_Bi_Bj exactly.
    const int64_t t0 = 43000000000ll;
    F4Rig rig;
    rig.yaw_rate = 0.9;
    rig.v = Eigen::Vector3d(0.8, -0.2, 0.1);
    rig.T_B_L = se3YawT(0.25, 0.9, 0.05, 0.02);

    const auto scenario = [&](ShadowTimeline& t) {
        t.setT_B_L(rig.T_B_L);
        oneIntervalScenario(t, t0, [&](ShadowTimeline& x, int64_t b) { rig.feed(x, b); });
    };
    const double measured_trans = measureError(scenario, false);
    const double measured_rot = measureError(scenario, true);

    // correct pipeline: interpolate T_W_L at the anchor times, normalise to the
    // body frame, then difference
    const gtsam::Vector6 tangent = gtsam::Pose3::Logmap(rig.measurement(0.0, 0.1));
    std::cout << "[F4] combined SE(3): measured trans " << measured_trans
              << " (expected " << tangent.tail<3>().norm() << "), measured rot "
              << measured_rot << " (expected " << tangent.head<3>().norm() << ")"
              << std::endl;
    // Near-exactness up to the contract-mandated interpolation error: with a
    // curved translation path the signed design's translation-LERP + slerp rule
    // has a chord-vs-arc gap (~3e-5 m per 0.06 s bracket; observed gate
    // innovation offset 1.2e-4 vs the analytic subgroup pose). This tolerance
    // still excludes raw-differencing (1.1e-2) and lever-arm variants (~0.9).
    EXPECT_NEAR(measured_trans, tangent.tail<3>().norm(), 5e-4);
    EXPECT_NEAR(measured_rot, tangent.head<3>().norm(), 1e-6);

    // tempting-but-wrong alternatives must be measurably different
    const Sophus::SE3d dT_lio = rig.T_W_L(0.0).inverse() * rig.T_W_L(0.1);
    const gtsam::Vector6 t_raw = gtsam::Pose3::Logmap(toGtsam(dT_lio));
    const gtsam::Vector6 t_left = gtsam::Pose3::Logmap(toGtsam(rig.T_B_L * dT_lio));
    const gtsam::Vector6 t_right = gtsam::Pose3::Logmap(toGtsam(dT_lio * rig.T_B_L));
    std::cout << "[F4] raw-differencing trans " << t_raw.tail<3>().norm()
              << ", lever-arm-left trans " << t_left.tail<3>().norm()
              << ", lever-arm-right trans " << t_right.tail<3>().norm() << std::endl;
    EXPECT_GT(std::abs(t_raw.tail<3>().norm() - measured_trans), 5e-3);
    EXPECT_GT(std::abs(t_left.tail<3>().norm() - measured_trans), 0.05);
    EXPECT_GT(std::abs(t_right.tail<3>().norm() - measured_trans), 0.05);
    EXPECT_GT(std::abs(t_left.head<3>().norm() - measured_rot), 0.02);
    EXPECT_GT(std::abs(t_right.head<3>().norm() - measured_rot), 0.02);
}


// ===========================================================================
// F5 - interpolation edge cases (no thresholds are modified for the fix)
// ===========================================================================
namespace
{

std::unique_ptr<ShadowTimeline> buildLioCase(
    const std::vector<std::pair<double, double>>& offs_yaw, int64_t t0)
{
    auto tl = std::make_unique<ShadowTimeline>(imuParams(), defaultCfg());
    feedRestImu(*tl, t0, 0.3, 200);
    for (const auto& s : offs_yaw)
    {
        tl->feedLioPose(t0 + static_cast<int64_t>(std::llround(s.first * 1e9)),
                        se3YawT(s.second, s.first, 0.0, 0.0));
    }
    return tl;
}

}  // namespace

TEST(F5_EdgeCases, BracketGapAndRotationBoundaries)
{
    const int64_t t0 = 44000000000ll;
    const int64_t anchor = t0 + 100000000ll;
    Sophus::SE3d T;

    // only a left bracket / only a right bracket / neither -> reject
    EXPECT_FALSE(buildLioCase({{0.05, 0.0}, {0.08, 0.0}}, t0)->lookupLioPoseAt(0, anchor, T));
    EXPECT_FALSE(buildLioCase({{0.12, 0.0}, {0.14, 0.0}}, t0)->lookupLioPoseAt(0, anchor, T));
    EXPECT_FALSE(buildLioCase({{0.2, 0.0}}, t0)->lookupLioPoseAt(0, anchor, T));

    // exact anchor sample -> accepted and exact (alpha boundary case)
    {
        auto tl = buildLioCase({{0.05, 0.1}, {0.1, 0.42}}, t0);
        ASSERT_TRUE(tl->lookupLioPoseAt(0, anchor, T));
        EXPECT_NEAR(T.translation().x(), 0.10, 1e-12);   // pose x of the 0.1 s
        // sample: buildLioCase encodes x = t, so the exact sample at 0.1 s has
        // x = 0.10 while its yaw is 0.42 (checked above).
        EXPECT_NEAR(gtsam::Rot3(T.rotationMatrix()).yaw(), 0.42, 1e-12);
    }

    // span just below / just above 0.15 s
    {
        auto below = buildLioCase({{0.025, 0.0}, {0.174999999, 0.0}}, t0);
        EXPECT_TRUE(below->lookupLioPoseAt(0, anchor, T));
        auto above = buildLioCase({{0.025, 0.0}, {0.175000001, 0.0}}, t0);
        EXPECT_FALSE(above->lookupLioPoseAt(0, anchor, T));
    }

    // CHARACTERISATION of the exact boundary: span == 0.15 s exactly, i.e. equal
    // to max_interpolation_gap_sec. The signed rule is a span *limit*
    // (0.15 s initial value), so equality is expected to be accepted.
    {
        auto exact = buildLioCase({{0.025, 0.0}, {0.175, 0.0}}, t0);
        const bool accepted = exact->lookupLioPoseAt(0, anchor, T);
        const double span = 150000000.0 * 1e-9;
        std::cout << "[F5] exact-boundary 0.15 s span: accepted=" << accepted
                  << ", span as double=" << span << ", span>0.15 -> " << (span > 0.15)
                  << std::endl;
        EXPECT_TRUE(accepted)
            << "150 ms span rejected: the <= 0.15 s limit became exclusive through "
               "ns->double conversion";
    }

    // rotation near 180 deg: SLERP must take the shortest path
    {
        const double d = M_PI - 0.0017453292519943296;   // 179.9 deg
        auto tl = buildLioCase({{0.05, -d}, {0.15, d}}, t0);
        ASSERT_TRUE(tl->lookupLioPoseAt(0, anchor, T));
        const double yaw = gtsam::Rot3(T.rotationMatrix()).yaw();
        EXPECT_TRUE(std::isfinite(yaw));
        EXPECT_NEAR(std::fabs(yaw), M_PI, 1e-6);
    }

    // very small rotation
    {
        auto tl = buildLioCase({{0.05, 0.0}, {0.15, 1e-7}}, t0);
        ASSERT_TRUE(tl->lookupLioPoseAt(0, anchor, T));
        EXPECT_NEAR(gtsam::Rot3(T.rotationMatrix()).yaw(), 5e-8, 1e-9);
    }

    // determinism: repeated queries on one instance return identical values
    {
        auto tl = buildLioCase({{0.025, 0.0}, {0.175, 0.0}}, t0);
        Sophus::SE3d A, B;
        const bool a = tl->lookupLioPoseAt(0, anchor, A);
        const bool b = tl->lookupLioPoseAt(0, anchor, B);
        EXPECT_EQ(a, b);
        if (a && b) EXPECT_TRUE(A.matrix() == B.matrix());
    }
}


// ===========================================================================
// F6 - A2 right-sample ZOH semantics (every subsegment, not only the terminal)
// ===========================================================================
TEST(F6_RightZoh, AccelerationStepAcrossAnchorBoundary)
{
    // interval 0 = [0, 0.1 s]; samples at 0.0 (a=1), 0.05 (a=1, exactly on the
    // interior anchor boundary -- the max_imu_dt_sec=0.05 bound is inclusive,
    // as pinned by F7) and 0.105 (a=20, the first sample at/after t_j).
    // right-sample ZOH => the sample at 0.05 owns (0, 0.05] and the terminal
    // subsegment (0.05, 0.1] is integrated with the 0.105 sample:
    // dV_x = 1*t1 + 20*t2, dP_x = 0.5*1*t1^2 + (1*t1)*t2 + 0.5*20*t2^2 with
    // t1 = t2 = 0.05 s. Left-sample ownership would give dV_x = 0.1 m/s and
    // dP_x = 0.005 m.
    ShadowTimeline tl(imuParams(), defaultCfg());
    const int64_t t0 = 45000000000ll;
    const int64_t t_mid = t0 + 50000000ll;
    const int64_t t_right = t0 + 105000000ll;
    const gtsam::Vector3 acc_low(1.0, 0.0, kGravity);
    const gtsam::Vector3 acc_high(20.0, 0.0, kGravity);
    const gtsam::Vector3 gyro0 = gtsam::Vector3::Zero();
    tl.feedImu(t0, acc_low, gyro0);
    tl.feedImu(t_mid, acc_low, gyro0);
    tl.feedImu(t_right, acc_high, gyro0);
    std::vector<ImuRec> recs{{t0, acc_low, gyro0},
                             {t_mid, acc_low, gyro0},
                             {t_right, acc_high, gyro0}};

    ASSERT_EQ(tl.diag().intervals_closed, 1);
    EXPECT_NEAR(coveredSec(recs, t0, t0 + 100000000ll), 0.1, 1e-15);

    const double t1 = static_cast<double>(t_mid - t0) * 1e-9;
    const double t2 = static_cast<double>(t0 + 100000000ll - t_mid) * 1e-9;
    const double dv_right = 1.0 * t1 + 20.0 * t2;
    const double dp_right = 0.5 * 1.0 * t1 * t1 + (1.0 * t1) * t2 + 0.5 * 20.0 * t2 * t2;
    const double dv_left = 1.0 * 0.1;
    const double dp_left = 0.5 * 1.0 * 0.01;

    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.anchorState(1, T, v, b));
    std::cout << "[F6] accel step: v_x = " << v.x() << " (right-ZOH " << dv_right
              << ", left-ZOH " << dv_left << "), p_x = " << T.translation().x()
              << " (right-ZOH " << dp_right << ", left-ZOH " << dp_left << ")"
              << std::endl;
    EXPECT_NEAR(v.x(), dv_right, 1e-2);
    EXPECT_NEAR(T.translation().x(), dp_right, 1e-3);
    EXPECT_GT(std::abs(dv_right - dv_left), 0.5);      // discriminating power

    const ZohOracle right = zohOracle(recs, t0, t0 + 100000000ll, 0.05, false);
    const ZohOracle left = zohOracle(recs, t0, t0 + 100000000ll, 0.05, true);
    ASSERT_TRUE(right.complete);
    EXPECT_NEAR(tl.imuRef(0).translation().x(),
                right.state.pose().translation().x(), 1e-12);
    EXPECT_GT(std::abs(tl.imuRef(0).translation().x() -
                       left.state.pose().translation().x()), 1e-3);
}


TEST(F6_RightZoh, RotationStepInInteriorAndTerminalSegments)
{
    // Hand-computed per-subsegment budgets (independent analytic oracle).
    // samples: 0.000(1.0) 0.020(1.0) 0.040(1.0) 0.060(3.0) 0.080(3.0)
    //          0.090(3.0) 0.098(5.0) 0.115(7.0)      [rad/s yaw rate]
    // right-sample ownership:
    //   (0,0.02]@1.0 + (0.02,0.04]@1.0 + (0.04,0.06]@3.0 + (0.06,0.08]@3.0
    //   + (0.08,0.09]@3.0 + (0.09,0.098]@5.0 + (0.098,0.1]@7.0
    //   = 0.02+0.02+0.06+0.06+0.03+0.04+0.014 = 0.244 rad
    // interior segments left-owned, terminal right-owned = 0.188 rad
    // everything left-owned = 0.184 rad
    ShadowTimeline tl(imuParams(), defaultCfg());
    const int64_t t0 = 46000000000ll;
    const gtsam::Vector3 accz(0.0, 0.0, kGravity);
    const double ts_ms[8] = {0.0, 20.0, 40.0, 60.0, 80.0, 90.0, 98.0, 115.0};
    const double rate[8] = {1.0, 1.0, 1.0, 3.0, 3.0, 3.0, 5.0, 7.0};
    std::vector<ImuRec> recs;
    for (int i = 0; i < 8; ++i)
    {
        const int64_t t = t0 + static_cast<int64_t>(ts_ms[i] * 1000000.0);
        const gtsam::Vector3 g(0.0, 0.0, rate[i]);
        tl.feedImu(t, accz, g);
        recs.push_back({t, accz, g});
    }
    ASSERT_EQ(tl.diag().intervals_closed, 1);

    const double yaw_impl = tl.imuRef(0).rotation().yaw();
    std::cout << "[F6] rotation step: yaw = " << yaw_impl
              << " (right-ZOH 0.244, mixed 0.188, left-ZOH 0.184)" << std::endl;
    EXPECT_NEAR(yaw_impl, 0.244, 1e-6);
    EXPECT_GT(std::abs(yaw_impl - 0.188), 0.03)
        << "interior subsegments are not right-sample owned";
    EXPECT_GT(std::abs(yaw_impl - 0.184), 0.03);

    const ZohOracle right = zohOracle(recs, t0, t0 + 100000000ll, 0.05, false);
    ASSERT_TRUE(right.complete);
    EXPECT_NEAR(tl.imuRef(0).rotation().yaw(), right.state.pose().rotation().yaw(), 1e-12);

    // orientation, velocity and position of the solved anchor
    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.anchorState(1, T, v, b));
    EXPECT_NEAR(gtsam::Rot3(T.rotation()).yaw(), 0.244, 1e-4);
    EXPECT_LT(v.norm(), 1e-6);
    EXPECT_LT(T.translation().norm(), 1e-9);
    EXPECT_NEAR(coveredSec(recs, t0, t0 + 100000000ll), 0.1, 1e-15);
}


// ===========================================================================
// F7 - continuous IMU coverage / gaps
// ===========================================================================
TEST(F7_ImuGaps, DtBoundaryAcceptRejectIsDeterministic)
{
    const int64_t t0 = 47000000000ll;
    const gtsam::Vector3 acc(0.0, 0.0, kGravity);
    const gtsam::Vector3 gyro = gtsam::Vector3::Zero();
    // probes the first internal dt of interval 0; the other subsegments stay
    // well inside the bound so that only the probed dt can trigger a gap.
    auto closed = [&](int64_t probed_dt) {
        ShadowTimeline tl(imuParams(), defaultCfg());   // max_imu_dt_sec = 0.05
        tl.feedImu(t0, acc, gyro);
        tl.feedImu(t0 + probed_dt, acc, gyro);
        tl.feedImu(t0 + 55000000ll, acc, gyro);
        tl.feedImu(t0 + 105000000ll, acc, gyro);
        return tl.diag().intervals_closed;
    };
    EXPECT_EQ(closed(30000000ll), 1);        // 30 ms  < 50 ms
    EXPECT_EQ(closed(49999999ll), 1);        // 49.999999 ms < 50 ms
    EXPECT_EQ(closed(50000000ll), 1);        // exactly 50 ms -> inclusive bound
    EXPECT_EQ(closed(50000001ll), 0);        // 50.000001 ms -> internal gap, no close
    EXPECT_EQ(closed(90000000ll), 0);        // 90 ms -> internal gap, no close
}

TEST(F7_ImuGaps, MultipleBadGapsThenValidRegion)
{
    ShadowConfig c = defaultCfg();
    c.max_imu_dt_sec = 0.02;                 // 20 ms, as in the R3 gap test
    ShadowTimeline tl(imuParams(), c);
    const int64_t t0 = 48000000000ll;
    const gtsam::Vector3 acc(0.0, 0.0, kGravity);
    const gtsam::Vector3 gyro = gtsam::Vector3::Zero();

    tl.feedImu(t0, acc, gyro);
    tl.feedImu(t0 + 10000000ll, acc, gyro);       // valid 10 ms
    tl.feedImu(t0 + 60000000ll, acc, gyro);       // 50 ms > 20 ms internal gap
    tl.feedImu(t0 + 105000000ll, acc, gyro);      // crosses t_1: interval 0 not closed
    ASSERT_EQ(tl.diag().intervals_closed, 0);
    ASSERT_EQ(tl.anchorCount(), 2);
    ASSERT_GT(tl.diag().imu_dropped, 0);

    // a completely valid region covering intervals 1 and 2 follows
    bool threw = false;
    std::string what;
    try
    {
        feedImuTrain(tl, t0 + 110000000ll, 18, 5000000ll, acc, gyro);   // 0.110..0.200
    }
    catch (const std::exception& e) { threw = true; what = e.what(); }
    catch (...) { threw = true; what = "non std::exception"; }

    std::cout << "[F7] after gap: intervals_closed=" << tl.diag().intervals_closed
              << " anchors=" << tl.anchorCount()
              << " factors=" << tl.totalGraphFactors()
              << " threw=" << threw << " what=" << what << std::endl;

    EXPECT_FALSE(threw)
        << "closing the interval AFTER an unclosed (gapped) interval threw: " << what;
    // the unclosed interval must not be finalised and must not have advanced the
    // anchor chain with a partial prediction
    EXPECT_EQ(tl.diag().intervals_closed, 1);
    EXPECT_EQ(tl.totalGraphFactors(), 5);   // 3 priors + 1 ImuFactor + 1 bias factor
}

TEST(F7_ImuGaps, PlaceholderAnchorStateIsSilentlyReportedAsValid)
{
    // Same IMU stream with and without an internal gap. The gapped run leaves
    // interval 0 unclosed, i.e. anchor 1 keeps the placeholder identity state
    // pushed at anchor creation and is never inserted into the graph. Any query
    // that selects anchor 1 must therefore not pretend to have a state.
    ShadowConfig c = defaultCfg();
    c.max_imu_dt_sec = 0.02;
    const int64_t t0 = 49000000000ll;
    const gtsam::Vector3 acc(10.0, 0.0, kGravity);       // a_x = 10 m/s^2
    const gtsam::Vector3 gyro = gtsam::Vector3::Zero();

    // reference run: identical stream, no gap
    ShadowTimeline ref(imuParams(), c);
    feedImuTrain(ref, t0, 30, 5000000ll, acc, gyro);
    const auto ref_state = ref.propagateTo(t0 + 150000000ll);
    ASSERT_TRUE(ref_state.valid);
    std::cout << "[F7] reference (no gap) x(0.15 s) = "
              << ref_state.T_W_B.translation().x() << " (analytic 0.1125)" << std::endl;
    EXPECT_NEAR(ref_state.T_W_B.translation().x(), 0.1125, 1e-3);

    // gapped run
    ShadowTimeline tl(imuParams(), c);
    tl.feedImu(t0, acc, gyro);
    tl.feedImu(t0 + 30000000ll, acc, gyro);     // valid
    tl.feedImu(t0 + 80000000ll, acc, gyro);     // 50 ms > 20 ms -> internal gap
    tl.feedImu(t0 + 105000000ll, acc, gyro);    // creates anchor 1; interval 0 fails
    tl.feedImu(t0 + 110000000ll, acc, gyro);
    tl.feedImu(t0 + 120000000ll, acc, gyro);
    tl.feedImu(t0 + 130000000ll, acc, gyro);
    tl.feedImu(t0 + 155000000ll, acc, gyro);

    ASSERT_EQ(tl.diag().intervals_closed, 0);
    ASSERT_EQ(tl.anchorCount(), 2);

    gtsam::Pose3 T1;
    gtsam::Vector3 v1;
    gtsam::imuBias::ConstantBias b1;
    const bool anchor_ok = tl.anchorState(1, T1, v1, b1);
    const auto st = tl.propagateTo(t0 + 150000000ll);
    std::cout << "[F7] gapped: anchorState(1) ok=" << anchor_ok
              << " x=" << T1.translation().x()
              << " ; propagateTo(0.15) valid=" << st.valid
              << " x=" << st.T_W_B.translation().x() << std::endl;

    // the truth at 0.15 s is 0.1125 m (the EM state is not an option here)
    EXPECT_TRUE(st.valid);
    EXPECT_NEAR(st.T_W_B.translation().x(), 0.1125, 1e-2)
        << "state propagated from a never-solved placeholder anchor: silent "
           "advance of the central anchor chain after an IMU gap";
    EXPECT_NEAR(T1.translation().x(), 0.05, 1e-2)
        << "anchorState(k) returns the creation placeholder as if it were a state";
}

TEST(F7_ImuGaps, DuplicateAndOutOfOrderImuSamplesDoNotCorruptCoverage)
{
    const int64_t t0 = 51000000000ll;
    const gtsam::Vector3 acc(0.0, 0.0, kGravity);
    const gtsam::Vector3 gyro = gtsam::Vector3::Zero();

    ShadowTimeline clean(imuParams(), defaultCfg());
    feedImuTrain(clean, t0, 40, 5000000ll, acc, gyro);      // 0 .. 0.200 s

    ShadowTimeline messy(imuParams(), defaultCfg());
    for (int i = 0; i <= 40; ++i)
    {
        const int64_t t = t0 + static_cast<int64_t>(i) * 5000000ll;
        messy.feedImu(t, acc, gyro);
        if (i == 9 || i == 20) messy.feedImu(t, acc, gyro);        // duplicates
    }
    messy.feedImu(t0 + 43000000ll, acc, gyro);                     // out of order
    messy.feedImu(t0 + 189000000ll, acc, gyro);                    // out of order

    EXPECT_GE(messy.diag().imu_dropped, 4);
    EXPECT_EQ(messy.diag().intervals_closed, clean.diag().intervals_closed);
    EXPECT_EQ(messy.anchorCount(), clean.anchorCount());
    for (int k = 0; k < clean.diag().intervals_closed; ++k)
    {
        EXPECT_TRUE(messy.imuRef(k).equals(clean.imuRef(k), 1e-15))
            << "interval " << k;
    }
}



// ===========================================================================
// F8 - historical propagateTo() under dynamic motion
// ===========================================================================
TEST(F8_HistoricalPropagation, DynamicStateCorrespondsToRequestedMeasurementTime)
{
    const int64_t t0 = 52000000000ll;
    const double a = 2.0;                       // a_x = 2 m/s^2
    ShadowTimeline tl(imuParams(), defaultCfg());
    feedImuTrain(tl, t0, 250, 5000000ll, gtsam::Vector3(a, 0.0, kGravity),
                 gtsam::Vector3::Zero());       // 0 .. 1.25 s
    ASSERT_EQ(tl.anchorCount(), 13);
    ASSERT_EQ(tl.diag().intervals_closed, 12);

    auto check = [&](int64_t stamp, double t, bool expect_valid) {
        const auto st = tl.propagateTo(stamp);
        EXPECT_EQ(st.valid, expect_valid) << "t = " << t;
        if (!expect_valid) return;
        EXPECT_EQ(st.stamp_ns, stamp);
        EXPECT_NEAR(st.T_W_B.translation().x(), 0.5 * a * t * t, 2e-3) << "t = " << t;
        EXPECT_NEAR(st.v_W.x(), a * t, 5e-3) << "t = " << t;
        EXPECT_LT(std::abs(st.T_W_B.translation().y()), 1e-6);
        EXPECT_LT(std::abs(st.T_W_B.translation().z()), 1e-6);
        EXPECT_LT(gtsam::Pose3::Logmap(st.T_W_B).head<3>().norm(), 1e-6);
    };

    check(t0 - 1000000ll, -0.001, false);  // before the first anchor: explicit failure
    check(t0, 0.0, true);                  // exact first anchor
    check(t0 + 55000000ll, 0.055, true);
    check(t0 + 100000000ll, 0.100, true);  // exact second anchor
    check(t0 + 555000000ll, 0.555, true);
    check(t0 + 999000000ll, 0.999, true);
    check(t0 + 1200000000ll, 1.200, true); // latest anchor
    check(t0 + 1201000000ll, 1.201, true); // slightly after the latest anchor

    gtsam::Pose3 T5, T12;
    gtsam::Vector3 v;
    gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.anchorState(5, T5, v, b));
    ASSERT_TRUE(tl.anchorState(12, T12, v, b));
    const auto st = tl.propagateTo(t0 + 555000000ll);
    std::cout << "[F8] t=0.555: returned x = " << st.T_W_B.translation().x()
              << " ; base anchor 5 x = " << T5.translation().x()
              << " ; newest anchor 12 x = " << T12.translation().x() << std::endl;
    EXPECT_NEAR(st.T_W_B.translation().x(), 0.308025, 2e-3);
}


// ===========================================================================
// F9 - one-shot holes, scan cursor and factor identity
// ===========================================================================
TEST(F9_OneShot, OutOfOrderArrivalFillsHolesAndIsIdempotent)
{
    const int64_t t0 = 53000000000ll;
    ShadowTimeline tl(imuParams(), defaultCfg());
    feedRestImu(tl, t0, 0.5, 200);                 // anchors 0..5, intervals 0..4
    const Sophus::SE3d P;                          // static source gauge

    // k+2 arrives and is accepted before k
    tl.feedLioPose(t0 + 200000000ll, P);
    tl.feedLioPose(t0 + 300000000ll, P);
    EXPECT_TRUE(tl.isIntervalLioConstrained(2));
    EXPECT_FALSE(tl.isIntervalLioConstrained(1));
    EXPECT_FALSE(tl.isIntervalLioConstrained(0));
    EXPECT_EQ(tl.diag().lio_accepted, 1);

    // k arrives: the earlier holes must be filled, not skipped
    tl.feedLioPose(t0, P);
    tl.feedLioPose(t0 + 100000000ll, P);
    EXPECT_TRUE(tl.isIntervalLioConstrained(0));
    EXPECT_TRUE(tl.isIntervalLioConstrained(1));
    EXPECT_EQ(tl.diag().lio_accepted, 3);

    // k+1 arrives
    tl.feedLioPose(t0 + 400000000ll, P);
    EXPECT_TRUE(tl.isIntervalLioConstrained(3));
    EXPECT_EQ(tl.diag().lio_accepted, 4);
    // 3 priors + 5 ImuFactors + 5 bias factors + 4 LIO BetweenFactors
    EXPECT_EQ(tl.totalGraphFactors(), 17);

    // duplicate replay of every k: exactly one factor per (source, epoch, k)
    for (int k = 0; k <= 4; ++k)
        tl.feedLioPose(t0 + static_cast<int64_t>(k) * 100000000ll, P);
    EXPECT_EQ(tl.diag().lio_accepted, 4);
    EXPECT_EQ(tl.totalGraphFactors(), 17);
    EXPECT_EQ(tl.diag().lio_rejected_duplicate, 0);   // idempotent by identity

    // ---- new-epoch semantics (frozen header: identity = (source, epoch, k)) ----
    // CONTRACT ASSERTION (expected to fail on this revision, kept as evidence):
    // a key that was never inserted must not be reported as a duplicate merely
    // because the same INTERVAL is already constrained by epoch 0.
    const AcceptDecision same_k_new_epoch =
        tl.insertRelativeConstraint(ConstraintKey{0u, 1u, 0}, gtsam::Pose3());
    std::cout << "[F9] same k in a new epoch -> " << toCString(same_k_new_epoch)
              << " (contract: (source, epoch, k) identity)" << std::endl;
    EXPECT_NE(same_k_new_epoch, AcceptDecision::REJECT_DUPLICATE)
        << "epoch dimension collapsed: interval-level pinning rejected a "
           "never-inserted (source, epoch, k) key";

    // a genuinely new interval in the new epoch is still accepted. Pair
    // 0.39/0.51 brackets interval 4 [0.4, 0.5] with a 0.12 s interpolation gap
    // (<= 0.15 s); a wider pair (0.35/0.55) would be refused by the
    // interpolation-gap limit, not by any one-shot policy.
    const int accepted_before_epoch1 = tl.diag().lio_accepted;
    tl.feedLioPose(t0 + 390000000ll, P, 1);
    tl.feedLioPose(t0 + 510000000ll, P, 1);
    EXPECT_TRUE(tl.isIntervalLioConstrained(4));
    EXPECT_TRUE(tl.hasConstraint(ConstraintKey{0u, 1u, 4}));
    EXPECT_FALSE(tl.hasConstraint(ConstraintKey{0u, 0u, 4}));
    EXPECT_EQ(tl.diag().lio_accepted, accepted_before_epoch1 + 1);
}

TEST(F9_OneShot, GateRejectedIntervalIsPinned_DocumentedPolicy)
{
    // CHARACTERISATION of the one-shot policy: a terminal gate rejection pins
    // the interval, so a later valid attempt for the SAME interval is not
    // retried. F9 explicitly asks whether a rejected interval may block a future
    // valid attempt, so the observed behaviour is pinned here.
    const int64_t t0 = 54000000000ll;
    ShadowTimeline tl(imuParams(), defaultCfg());
    feedRestImu(tl, t0, 0.3, 200);

    tl.feedLioPose(t0, se3YawT(0.0, 0.0, 0.0, 0.0));
    tl.feedLioPose(t0 + 100000000ll, se3YawT(0.0, 3.0, 0.0, 0.0));   // 3 m outlier
    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 1);
    EXPECT_TRUE(tl.isIntervalLioConstrained(0));

    // corrected samples for the very same interval arrive afterwards
    tl.feedLioPose(t0 + 200000000ll, se3YawT(0.0, 0.0, 0.0, 0.0));
    tl.feedLioPose(t0 + 300000000ll, se3YawT(0.0, 0.0, 0.0, 0.0));
    std::cout << "[F9] after corrected samples: accepted=" << tl.diag().lio_accepted
              << " rejected_dup=" << tl.diag().lio_rejected_duplicate
              << " interval0 constrained=" << tl.isIntervalLioConstrained(0) << std::endl;
    EXPECT_EQ(tl.diag().lio_accepted, 1);       // only interval 1 gets constrained
    EXPECT_FALSE(tl.hasConstraint(ConstraintKey{0u, 0u, 0}));
}


// ===========================================================================
// F10 - the immutable reference stays pre-fusion
// ===========================================================================
namespace
{

// The production IMU parameters are ~1e4 times stiffer than the LIO factor
// (accel covariance 1e-6 -> ~5e-6 m sigma over a 0.1 s interval vs 0.05 m for
// the source factor), so with them no source factor can move the posterior
// materially. To make "a large posterior state change" achievable at all, this
// rig softens the IMU noise density and sharpens the source factor.
std::shared_ptr<gtsam::PreintegrationParams> softImuParams()
{
    auto p = gtsam::PreintegrationParams::MakeSharedU(kGravity);
    p->accelerometerCovariance = gtsam::I_3x3 * 1.0;
    p->gyroscopeCovariance = gtsam::I_3x3 * 1.0;
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;
    return p;
}

ShadowConfig sharpSourceCfg()
{
    ShadowConfig c = defaultCfg();
    c.lio_sigma_rot_rad = 0.001;
    c.lio_sigma_trans_m = 0.001;
    return c;
}

}  // namespace

TEST(F10_ImmutableReference, PreFusionReferenceIsNeverRecomputed)
{
    // The big factor is inserted for the LATEST interval (9 = [0.9, 1.0 s], zero
    // lateness at the end of the stream). Constrained interval identity is
    // irrelevant to what F10 verifies: the frozen pre-fusion reference must not
    // be recomputed from the perturbed posterior, whatever interval is used.
    const int64_t t0 = 55000000000ll;
    ShadowTimeline pure(softImuParams(), sharpSourceCfg());   // IMU only
    std::vector<ImuRec> recs;
    feedRestImu(pure, t0, 1.0, 200, &recs);
    ShadowTimeline pert(softImuParams(), sharpSourceCfg());   // IMU + big source factor
    for (const auto& r : recs) pert.feedImu(r.t, r.acc, r.gyro);
    ASSERT_EQ(pure.diag().intervals_closed, 10);
    ASSERT_EQ(pert.diag().intervals_closed, 10);

    const gtsam::Pose3 ref9_before = pert.imuRef(9);
    const gtsam::Pose3 big(gtsam::Rot3(), gtsam::Point3(0.9, 0.0, 0.0));
    ASSERT_EQ(pert.insertRelativeConstraint(ConstraintKey{0u, 0u, 9}, big),
              AcceptDecision::ACCEPTED);

    gtsam::Pose3 A9, A10, P9, P10;
    gtsam::Vector3 v;
    gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(pert.anchorState(9, A9, v, b));
    ASSERT_TRUE(pert.anchorState(10, A10, v, b));
    ASSERT_TRUE(pure.anchorState(9, P9, v, b));
    ASSERT_TRUE(pure.anchorState(10, P10, v, b));
    const double posterior_shift = (A10.translation() - P10.translation()).norm();
    const double rel_after = A9.between(A10).translation().x();
    std::cout << "[F10] posterior anchor10 shift = " << posterior_shift
              << " m ; posterior relative x = " << rel_after
              << " ; frozen ref9 x = " << pert.imuRef(9).translation().x() << std::endl;

    EXPECT_GT(posterior_shift, 0.3);                           // posterior really moved
    EXPECT_TRUE(pert.imuRef(9).equals(ref9_before, 1e-15));     // ref unchanged
    EXPECT_GT(std::abs(rel_after - pert.imuRef(9).translation().x()), 0.3);
    // no later helper recomputes a reference from optimized states
    for (int k = 0; k < pert.diag().intervals_closed; ++k)
        EXPECT_TRUE(pert.imuRef(k).equals(pure.imuRef(k), 1e-15)) << "interval " << k;
}


// ===========================================================================
// F11 - noise / tangent order regression
// ===========================================================================
TEST(F11_NoiseOrder, AsymmetricSigmaSetWhitenstheCorrectBlock)
{
    auto model = makePoseNoise(0.01, 2.0);   // rot 0.01, trans 2.0
    const auto diag = std::dynamic_pointer_cast<gtsam::noiseModel::Diagonal>(model);
    ASSERT_TRUE(diag != nullptr);
    const gtsam::Vector sig = diag->sigmas();
    EXPECT_NEAR(sig(0), 0.01, 1e-15);
    EXPECT_NEAR(sig(2), 0.01, 1e-15);
    EXPECT_NEAR(sig(3), 2.0, 1e-15);
    EXPECT_NEAR(sig(5), 2.0, 1e-15);

    // a 1.0 m pure-translation mismatch must be whitened by 2.0 (order locked)
    gtsam::NonlinearFactorGraph g;
    g.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', 0), gtsam::Symbol('x', 1),
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)), model);
    gtsam::Values v;
    v.insert(gtsam::Symbol('x', 0), gtsam::Pose3());
    v.insert(gtsam::Symbol('x', 1), gtsam::Pose3());
    const double err_trans = g.error(v);
    EXPECT_NEAR(err_trans, 0.5 * (1.0 / 2.0) * (1.0 / 2.0), 1e-9);
    EXPECT_LT(err_trans, 1.0);   // swapped order would give 0.5*(1/0.01)^2 = 5000

    // a 0.5 rad pure-rotation mismatch must be whitened by 0.01
    gtsam::NonlinearFactorGraph g2;
    g2.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        gtsam::Symbol('x', 0), gtsam::Symbol('x', 1),
        gtsam::Pose3(gtsam::Rot3::Yaw(0.5), gtsam::Point3(0, 0, 0)), model);
    gtsam::Values v2;
    v2.insert(gtsam::Symbol('x', 0), gtsam::Pose3());
    v2.insert(gtsam::Symbol('x', 1), gtsam::Pose3());
    EXPECT_NEAR(g2.error(v2), 0.5 * 50.0 * 50.0, 1e-4);
}

TEST(F11_GateTangentOrder, LogmapRotationThenTranslationBlocks)
{
    const int64_t t0 = 56000000000ll;
    ShadowTimeline tl(imuParams(), defaultCfg());
    feedRestImu(tl, t0, 0.5, 200);   // intervals 0..4 closed, refs are identity

    // rotation-only violation -> the ROTATION block must catch it
    EXPECT_EQ(tl.insertRelativeConstraint(
                  ConstraintKey{0u, 0u, 0},
                  gtsam::Pose3(gtsam::Rot3::Yaw(0.40), gtsam::Point3(0, 0, 0))),
              AcceptDecision::REJECT_INNOVATION_ROT);
    // translation-only violations
    EXPECT_EQ(tl.insertRelativeConstraint(
                  ConstraintKey{0u, 0u, 1},
                  gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.5, 0.0, 0.0))),
              AcceptDecision::ACCEPTED);
    EXPECT_EQ(tl.insertRelativeConstraint(
                  ConstraintKey{0u, 0u, 2},
                  gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.2, 0.0, 0.0))),
              AcceptDecision::REJECT_INNOVATION_TRANS);
    // combined violation is reported by the translation block (checked first)
    EXPECT_EQ(tl.insertRelativeConstraint(
                  ConstraintKey{0u, 0u, 3},
                  gtsam::Pose3(gtsam::Rot3::Yaw(0.40), gtsam::Point3(1.2, 0.0, 0.0))),
              AcceptDecision::REJECT_INNOVATION_TRANS);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_rot, 1);
    EXPECT_EQ(tl.diag().lio_rejected_innovation_trans, 2);
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}


// ===========================================================================
// F12 - graph lifecycle regression
// ===========================================================================
TEST(F12_Lifecycle, ExactFactorAccountingGaugePriorAndEpochStability)
{
    const int64_t t0 = 57000000000ll;
    const gtsam::Vector3 accz(0.0, 0.0, kGravity);
    const gtsam::Vector3 gyro0 = gtsam::Vector3::Zero();
    const Sophus::SE3d P;
    ShadowTimeline tl(imuParams(), defaultCfg());

    tl.feedImu(t0, accz, gyro0);
    tl.feedLioPose(t0, P);
    const int n = 50;
    for (int k = 0; k < n; ++k)
    {
        for (int i = 1; i <= 20; ++i)
            tl.feedImu(t0 + static_cast<int64_t>(k * 20 + i) * 5000000ll, accz, gyro0);
        tl.feedLioPose(t0 + static_cast<int64_t>(k + 1) * 100000000ll, P);
    }

    EXPECT_EQ(tl.anchorCount(), 51);          // > 32 anchors retained, no eviction
    EXPECT_EQ(tl.diag().intervals_closed, n);
    EXPECT_EQ(tl.diag().lio_accepted, n);
    EXPECT_EQ(tl.totalGraphFactors(),
              3u + 2u * static_cast<size_t>(n) + static_cast<size_t>(n));
    EXPECT_EQ(tl.diag().pose_priors_added, 1);
    EXPECT_EQ(tl.diag().vel_priors_added, 1);
    EXPECT_EQ(tl.diag().bias_priors_added, 1);
    for (int k = 0; k <= n; ++k)
    {
        gtsam::Pose3 T;
        gtsam::Vector3 v;
        gtsam::imuBias::ConstantBias b;
        ASSERT_TRUE(tl.anchorState(k, T, v, b)) << "anchor " << k;
    }

    // a source epoch change must not reset or shrink the central graph
    const size_t factors_before = tl.totalGraphFactors();
    tl.feedLioPose(t0 + 5100000000ll, P, 1);
    tl.feedLioPose(t0 + 5200000000ll, P, 1);
    EXPECT_GE(tl.totalGraphFactors(), factors_before);
    EXPECT_EQ(tl.anchorCount(), 51);
    EXPECT_EQ(tl.diag().pose_priors_added, 1);
    EXPECT_EQ(tl.diag().lio_accepted, n);
}

TEST(F12_Lifecycle, ExternalFactorsRemainAdjacentAnchorPairs)
{
    const int64_t t0 = 58000000000ll;
    ShadowTimeline tl(softImuParams(), sharpSourceCfg());
    feedRestImu(tl, t0, 1.0, 200);              // 11 anchors, 10 intervals
    ASSERT_EQ(tl.diag().intervals_closed, 10);

    std::vector<gtsam::Pose3> before, after;
    auto snapshot = [&](std::vector<gtsam::Pose3>& out) {
        out.clear();
        for (int k = 0; k <= 9; ++k)
        {
            gtsam::Pose3 A, B;
            gtsam::Vector3 v;
            gtsam::imuBias::ConstantBias b;
            const bool oka = tl.anchorState(k, A, v, b);
            const bool okb = tl.anchorState(k + 1, B, v, b);
            EXPECT_TRUE(oka && okb);
            out.push_back(A.between(B));
        }
    };
    snapshot(before);
    ASSERT_EQ(tl.insertRelativeConstraint(
                  ConstraintKey{0u, 0u, 5},
                  gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.08, 0.0, 0.0))),
              AcceptDecision::ACCEPTED);
    snapshot(after);

    std::cout << "[F12] constrained interval 5 relative displacement change = "
              << transErr(before[5], after[5]) << std::endl;
    EXPECT_GT(transErr(before[5], after[5]), 0.05);

    // CHARACTERISATION: the signed contract is silent on posterior locality.
    // One 0.08 m source factor on (X_5, X_6) shifts the *relative measurements*
    // of non-adjacent intervals through the coupled least-squares solve
    // (observed: monotone decay away from k = 5, up to ~0.06 m / 0.07 rad at
    // k = 0). Recorded as evidence for the review's risk section; only a
    // generous sanity bound is asserted, not an invented immutability policy.
    std::cout << "[F12] interval relative-measurement changes:" << std::endl;
    for (int k = 0; k <= 9; ++k)
        std::cout << "  k=" << k << " dtrans=" << transErr(before[k], after[k])
                  << " drot=" << rotErr(before[k], after[k]) << std::endl;
    for (int k = 0; k <= 9; ++k)
    {
        if (k == 5) continue;
        EXPECT_LT(transErr(before[k], after[k]), 0.25)
            << "coupling magnitude sanity bound (k = " << k << ")";
    }
}

}  // namespace
