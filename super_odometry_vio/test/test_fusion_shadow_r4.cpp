#include <gtest/gtest.h>



#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"

namespace {
using namespace super_odometry_vio::fusion_2021;
ShadowConfig baseConfig() {
    ShadowConfig c;
    c.anchor_rate_hz = 10.0;
    c.max_constraint_lateness_ns = 500000000ll;
    c.max_imu_dt_ns = 50000000ll;
    c.source_reorder_horizon_ns = 100000000ll;
    c.max_interpolation_gap_ns = 150000000ll;
    
    return c;
}
std::shared_ptr<gtsam::PreintegrationParams> imuParams() {
    auto p = gtsam::PreintegrationParams::MakeSharedU(9.81);
    p->accelerometerCovariance = gtsam::I_3x3 * 1e-6;
    p->gyroscopeCovariance = gtsam::I_3x3 * 1e-6;
    p->integrationCovariance = gtsam::I_3x3 * 1e-8;
    return p;
}

// 1. GapDoesNotCreateValidPlaceholderAnchor
TEST(AnchorGap, GapDoesNotCreateValidPlaceholderAnchor) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 10000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    // gap > 50ms
    tl.feedImu(t0 + 70000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 110000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    EXPECT_EQ(tl.anchorStatus(1), AnchorStatus::INVALID_GAP);
}

// 2. LatestAnchorIgnoresScheduledUnsolvedAnchor
TEST(AnchorGap, LatestAnchorIgnoresScheduledUnsolvedAnchor) {
    // Feed enough IMU to close some intervals (making those anchors SOLVED),
    // then verify latestAnchor returns the latest SOLVED anchor (not any
    // SCHEDULED anchor beyond the IMU horizon).
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    // 22 IMU at 5 ms spacing = 110 ms total. anchor_rate_hz=10 → dt_a=100ms.
    // Creates anchors 0 and 1. closeInterval(0) fires, flushing the
    // optimizer which marks anchor 0 SOLVED.  Anchor 1 is GRAPH_INSERTED
    // and also becomes SOLVED in the same flush.
    for(int i=0; i<=22; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    // anchorCount() >= 2 means at least one interval was closed.
    ASSERT_GE(tl.anchorCount(), 2);
    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.latestAnchor(T, v, b));
    // Gravity-only static rig → translation should be near zero.
    EXPECT_NEAR(T.translation().norm(), 0.0, 1e-4);
}

// 3. AnchorStateRejectsInvalidOrUnsolvedAnchor
TEST(AnchorGap, AnchorStateRejectsInvalidOrUnsolvedAnchor) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 110000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    EXPECT_FALSE(tl.anchorState(1, T, v, b));
}

// 4. GapIntervalDoesNotInsertMissingKeyFactor
TEST(AnchorGap, GapIntervalDoesNotInsertMissingKeyFactor) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 10000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 80000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 110000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    // total graph factors should be 3 priors, no between factors
    
    // total graph factors should be 0 because graph is not flushed for INVALID_GAP
    EXPECT_EQ(tl.totalGraphFactors(), 0);
}

// 5. PropagateToGapReturnsInvalid
TEST(AnchorGap, PropagateToGapReturnsInvalid) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 10000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    // Propagate to a gap
    tl.feedImu(t0 + 80000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    auto out = tl.propagateTo(t0 + 90000000ll);
    EXPECT_FALSE(out.valid);
}

// 6. PermutationWithinReorderHorizonCommitsSameMeasurement

TEST(AnchorGap, GapBreaksFusionSegmentUntilExplicitReset) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    
    // 1. feed continuous IMU to create several SOLVED anchors
    int64_t t = t0;
    for (int i = 0; i < 70; ++i) { // 350ms total
        tl.feedImu(t, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        t += 5000000ll;
    }
    
    bool valid = false;
    AnchorState pre_gap_latest = tl.latestAnchor(&valid);
    EXPECT_TRUE(valid);
    int pre_gap_k = pre_gap_latest.k;

    // 2. introduce dt > max_imu_dt_ns
    int64_t gap_t = t + 100000000ll; // dt = 100ms > max_imu_dt_ns (50ms)

    // 3. feed >= 2 seconds of later continuous IMU
    int64_t t_post = gap_t;
    for (int i = 0; i < 420; ++i) { // 2100ms total
        tl.feedImu(t_post, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        t_post += 5000000ll;
    }

    // 4 & 5. assert latestAnchor does not advance past the last pre-gap solved anchor
    bool valid2 = false;
    AnchorState post_gap_latest = tl.latestAnchor(&valid2);
    EXPECT_TRUE(valid2);
    EXPECT_EQ(post_gap_latest.k, pre_gap_k);

    // 6. assert anchorState(post-gap) returns false
    bool state_valid = false;
    tl.anchorState(pre_gap_k + 5, &state_valid);
    EXPECT_FALSE(state_valid);

    // 7. assert propagateTo(post-gap query).valid == false
    auto out = tl.propagateTo(gap_t + 1000000000ll);
    EXPECT_FALSE(out.valid);
}

TEST(SourceFinalization, PermutationWithinReorderHorizonCommitsSameMeasurement) {
    ShadowConfig cfgA = baseConfig(); cfgA.source_reorder_horizon_ns = 300000000ll; ShadowTimeline tlA(imuParams(), cfgA);
    ShadowConfig cfgB = baseConfig(); cfgB.source_reorder_horizon_ns = 300000000ll; ShadowTimeline tlB(imuParams(), cfgB);
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) {
        tlA.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        tlB.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    }
    
    // Timeline A: chronologically
    tlA.feedLioPose(t0, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.00, 0, 0)));
    tlA.feedLioPose(t0 + 50000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.05, 0, 0)));
    tlA.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.10, 0, 0)));
    tlA.feedLioPose(t0 + 650000000ll, Sophus::SE3d()); // Finalizes t0+100ms
    
    // Timeline B: out of order
    tlB.feedLioPose(t0 + 50000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.05, 0, 0)));
    tlB.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.10, 0, 0)));
    tlB.feedLioPose(t0, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.00, 0, 0)));
    tlB.feedLioPose(t0 + 650000000ll, Sophus::SE3d()); // Finalizes t0+100ms
    
    EXPECT_EQ(tlA.diag().lio_accepted, 1);
    EXPECT_EQ(tlB.diag().lio_accepted, 1);
    
    gtsam::Pose3 measA, measB;
    ConstraintId key{0, 0, 0};
    ASSERT_TRUE(tlA.committedMeasurement(key, measA));
    ASSERT_TRUE(tlB.committedMeasurement(key, measB));
    EXPECT_TRUE(measA.equals(measB, 1e-6));
}

// 7. PermutationWithinReorderHorizonProducesSamePosterior
TEST(SourceFinalization, PermutationWithinReorderHorizonProducesSamePosterior) {
    ShadowTimeline tl1(imuParams(), baseConfig());
    ShadowTimeline tl2(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) {
        tl1.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        tl2.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    }
    
    tl1.feedLioPose(t0, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.00, 0, 0)));
    tl1.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.10, 0, 0)));
    tl1.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    
    tl2.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.10, 0, 0)));
    tl2.feedLioPose(t0, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.00, 0, 0)));
    tl2.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    
    gtsam::Pose3 T1, T2; gtsam::Vector3 v1, v2; gtsam::imuBias::ConstantBias b1, b2;
    ASSERT_TRUE(tl1.anchorState(1, T1, v1, b1));
    ASSERT_TRUE(tl2.anchorState(1, T2, v2, b2));
    
    EXPECT_TRUE(T1.equals(T2, 1e-6));
}

// 8. ExactAnchorSampleArrivingLaterBeforeWatermarkWins
TEST(SourceFinalization, ExactAnchorSampleArrivingLaterBeforeWatermarkWins) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    // Early imperfect bracket
    tl.feedLioPose(t0 + 110000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, 0, 0)));
    // Exact sample arrives later but before watermark finalizes it
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.14, 0, 0)));
    
    // Finalize
    tl.feedLioPose(t0 + 550000000ll, Sophus::SE3d());
    
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    
    // Check committed measurement directly using oracle
    gtsam::Pose3 meas;
    ConstraintId key{0, 0, 0};
    ASSERT_TRUE(tl.committedMeasurement(key, meas));
    EXPECT_NEAR(meas.translation().x(), 0.14, 1e-6); // Exact match
}

// 9. LateAfterWatermarkDroppedAndCounted
TEST(SourceFinalization, LateAfterWatermarkDroppedAndCounted) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedLioPose(t0 + 500000000ll, Sophus::SE3d()); // Watermark to 200ms
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d()); // 100ms < 200ms -> Dropped
    
    EXPECT_EQ(tl.diag().lio_late_after_watermark, 1);
}

// 10. NoCommitBeforeFinalizable
TEST(SourceFinalization, NoCommitBeforeFinalizable) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    
    // Max seen = 100ms -> Watermark = 0ms. Interval 1 (t=100ms) not finalizable!
    EXPECT_EQ(tl.diag().lio_accepted, 0);
    
    // Push max seen to 450ms -> Watermark 150ms -> Finalizable!
    tl.feedLioPose(t0 + 550000000ll, Sophus::SE3d());
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}

// 11. SameSourceSameKAcrossEpochRejectsSlotOccupied
TEST(SourceEpoch, SameSourceSameKAcrossEpochRejectsSlotOccupied) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d(), 0); // Finalizes k=0
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    
    // Now push new epoch manually with constraint
    ConstraintId key{0, 1, 0};
    AcceptDecision dec = tl.insertRelativeConstraint(key, gtsam::Pose3());
    EXPECT_EQ(dec, AcceptDecision::REJECT_SLOT_OCCUPIED);
}

// 12. SameIdentityReplayRejectsDuplicateIdentity
TEST(SourceEpoch, SameIdentityReplayRejectsDuplicateIdentity) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    ConstraintId key{0, 0, 0};
    tl.insertRelativeConstraint(key, gtsam::Pose3());
    AcceptDecision dec = tl.insertRelativeConstraint(key, gtsam::Pose3());
    EXPECT_EQ(dec, AcceptDecision::REJECT_DUPLICATE);
}

// 13. NewEpochFutureIntervalAllowed
TEST(SourceEpoch, NewEpochFutureIntervalAllowed) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    ConstraintId key1{0, 0, 0};
    tl.insertRelativeConstraint(key1, gtsam::Pose3());
    ConstraintId key2{0, 1, 1};
    AcceptDecision dec = tl.insertRelativeConstraint(key2, gtsam::Pose3());
    EXPECT_EQ(dec, AcceptDecision::ACCEPTED);
}

// 14. CrossEpochBracketRejected
TEST(SourceEpoch, CrossEpochBracketRejected) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 1);
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d(), 1);
    // Interval 0 bracket cross epoch (0 and 1) is rejected
    EXPECT_EQ(tl.diag().lio_rejected_cross_epoch, 1);
}

// 15. MaxInterpolationSpanExactBoundaryAccepted
TEST(TimeBoundary, MaxInterpolationSpanExactBoundaryAccepted) {
    ShadowTimeline tl(imuParams(), baseConfig());
    Sophus::SE3d out;
    tl.feedLioPose(1000000000ll, Sophus::SE3d());
    tl.feedLioPose(1150000000ll, Sophus::SE3d());
    // Query at 1050000000. Gap is exactly 150000000 (150ms).
    uint32_t ep; EXPECT_EQ(tl.lookupLioPoseAt(1050000000ll, out, ep), AcceptDecision::ACCEPTED);
}

// 16. MaxInterpolationSpanBoundaryPlusOneNsRejected
TEST(TimeBoundary, MaxInterpolationSpanBoundaryPlusOneNsRejected) {
    ShadowTimeline tl(imuParams(), baseConfig());
    Sophus::SE3d out;
    tl.feedLioPose(1000000000ll, Sophus::SE3d());
    tl.feedLioPose(1150000001ll, Sophus::SE3d());
    // Query at 1050000000. Gap is 150000001 ns.
    uint32_t ep; EXPECT_EQ(tl.lookupLioPoseAt(1050000000ll, out, ep), AcceptDecision::REJECT_NO_BRACKET);
}

// 17. LatenessExactBoundaryAccepted
TEST(TimeBoundary, LatenessExactBoundaryAccepted) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    // We want lateness = newest - t_j <= 500,000,000. 
    // t_j for k=0 is t0 + 100,000,000.
    // So newest can be up to t0 + 600,000,000.
    // Feed 120 IMU samples (600ms)
    for(int i=0; i<=120; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    ConstraintId key{0, 0, 0};
    EXPECT_EQ(tl.insertRelativeConstraint(key, gtsam::Pose3()), AcceptDecision::ACCEPTED);
}

// 18. LatenessBoundaryPlusOneNsRejected
TEST(TimeBoundary, LatenessBoundaryPlusOneNsRejected) {
    ShadowConfig c = baseConfig();
    c.max_constraint_lateness_ns = 499999999ll; // So 500M will reject
    ShadowTimeline tl(imuParams(), c);
    const int64_t t0 = 1000000000ll;
    // We want lateness = newest - t_j = 500,000,001. 
    // t_j for k=0 is t0 + 100,000,000.
    // So newest must be t0 + 600,000,001.
    // Feed 120 IMU samples (600ms), plus 1ns
    for(int i=0; i<=120; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 600000001ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    ConstraintId key{0, 0, 0};
    EXPECT_EQ(tl.insertRelativeConstraint(key, gtsam::Pose3()), AcceptDecision::REJECT_TOO_LATE);
}

// 19. CrossEpochBracketGetsExplicitTerminalReason
TEST(SourceEpoch, CrossEpochBracketGetsExplicitTerminalReason) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0 - 10000000ll, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 1); // Cross epoch!
    tl.feedLioPose(t0 + 350000000ll, Sophus::SE3d(), 1); // Finalizes t0
    
    // Interval 0 bracket cross epoch (0 and 1) is rejected
    EXPECT_EQ(tl.diag().lio_rejected_cross_epoch, 1);
}

// 20. NotFinalizableAtAnchorEndOnly
TEST(SourceFinalization, NotFinalizableAtAnchorEndOnly) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d()); 
    // Watermark is t0 (horizon 100ms). t_j = t0+100ms. Max interpolation gap = 150ms.
    // Influence horizon is t_j + max_gap = t0 + 250ms.
    // Current watermark < influence horizon. So not finalizable!
    EXPECT_EQ(tl.diag().lio_accepted, 0);
}

// 21. FinalizableAfterInterpolationInfluenceHorizon
TEST(SourceFinalization, FinalizableAfterInterpolationInfluenceHorizon) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d()); 
    // Feed one that pushes watermark to t_j + max_gap
    // t_j = t0 + 100ms. max_gap = 150ms. Influence horizon = t0 + 250ms.
    // To get watermark to 250ms (horizon 100ms), we need max_seen = 350ms.
    tl.feedLioPose(t0 + 550000000ll, Sophus::SE3d()); 
    
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}

// 22. CloserRightBracketArrivingAfterTjBeforeInfluenceHorizonWins
TEST(SourceFinalization, CloserRightBracketArrivingAfterTjBeforeInfluenceHorizonWins) {
    ShadowConfig cfg = baseConfig(); cfg.source_reorder_horizon_ns = 300000000ll; ShadowTimeline tl(imuParams(), cfg);
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=40; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0, 0, 0)));
    // right bracket at 240ms (gap 240ms). t_j = 100ms.
    tl.feedLioPose(t0 + 240000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.24, 0, 0))); 
    
    // Now watermark = 140ms. Influence horizon is 250ms. Not finalized yet.
    // Later sample at 160ms arrives!
    tl.feedLioPose(t0 + 150000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.15, 0, 0)));
    
    // Finalize
    tl.feedLioPose(t0 + 550000000ll, Sophus::SE3d()); 
    
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    
    gtsam::Pose3 meas;
    ConstraintId key{0, 0, 0};
    ASSERT_TRUE(tl.committedMeasurement(key, meas));
    // The measurement should be interpolated between 0 and 160ms.
    // at t=100ms, alpha = 100/160 = 0.625. x = 0.16 * 0.625 = 0.1.
    // 0.14 * (100 / 140) = 0.1
    EXPECT_NEAR(meas.translation().x(), 0.1, 1e-6);
}
}
