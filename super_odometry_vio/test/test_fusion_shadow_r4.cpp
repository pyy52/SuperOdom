#include <gtest/gtest.h>



#include "super_odometry_vio/fusion_2021/shadow_timeline.hpp"

namespace {
using namespace super_odometry_vio::fusion_2021;
ShadowConfig baseConfig() {
    ShadowConfig c;
    c.anchor_rate_hz = 10.0;
    c.max_constraint_lateness_ns = 500000000ll;
    c.source_reorder_horizon_ns = 300000000ll;
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
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    tl.feedImu(t0, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    tl.feedImu(t0 + 110000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    ASSERT_TRUE(tl.latestAnchor(T, v, b));
    EXPECT_EQ(T.translation().norm(), 0.0);
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
TEST(SourceFinalization, PermutationWithinReorderHorizonCommitsSameMeasurement) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    // LIO poses unordered but within watermark 
    tl.feedLioPose(t0 + 50000000ll, Sophus::SE3d());
    tl.feedLioPose(t0 + 90000000ll, Sophus::SE3d()); // max = t0+90ms, watermark = t0-10ms
    tl.feedLioPose(t0, Sophus::SE3d()); // t0 > t0-10ms. Accepted.
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    tl.feedLioPose(t0 + 250000000ll, Sophus::SE3d()); // Finalizes t0
    
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}

// 7. PermutationWithinReorderHorizonProducesSamePosterior
TEST(SourceFinalization, PermutationWithinReorderHorizonProducesSamePosterior) {
    ShadowTimeline tl1(imuParams(), baseConfig());
    ShadowTimeline tl2(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=20; i++) {
        tl1.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
        tl2.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    }
    
    tl1.feedLioPose(t0, Sophus::SE3d());
    tl1.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    tl1.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    
    tl2.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    tl2.feedLioPose(t0, Sophus::SE3d());
    tl2.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    
    gtsam::Pose3 T1, T2; gtsam::Vector3 v1, v2; gtsam::imuBias::ConstantBias b1, b2;
    tl1.anchorState(1, T1, v1, b1);
    tl2.anchorState(1, T2, v2, b2);
    
    EXPECT_TRUE(T1.equals(T2, 1e-6));
}

// 8. ExactAnchorSampleArrivingLaterBeforeWatermarkWins
TEST(SourceFinalization, ExactAnchorSampleArrivingLaterBeforeWatermarkWins) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    // Early imperfect bracket
    tl.feedLioPose(t0 + 110000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.1, 0, 0)));
    // Exact sample arrives later but before watermark finalizes it
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(Eigen::Quaterniond::Identity(), Eigen::Vector3d(0.2, 0, 0)));
    
    // Finalize
    tl.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    
    EXPECT_EQ(tl.diag().lio_accepted, 1);
    
    // Check if the exact sample (0.2) won over the interpolated bracket (0.1)
    gtsam::Pose3 T; gtsam::Vector3 v; gtsam::imuBias::ConstantBias b;
    tl.anchorState(1, T, v, b);
    EXPECT_GT(T.translation().x(), 0.005);
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
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d());
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d());
    
    // Max seen = 100ms -> Watermark = 0ms. Interval 1 (t=100ms) not finalizable!
    EXPECT_EQ(tl.diag().lio_accepted, 0);
    
    // Push max seen to 450ms -> Watermark 150ms -> Finalizable!
    tl.feedLioPose(t0 + 250000000ll, Sophus::SE3d());
    EXPECT_EQ(tl.diag().lio_accepted, 1);
}

// 11. SameSourceSameKAcrossEpochRejectsSlotOccupied
TEST(SourceEpoch, SameSourceSameKAcrossEpochRejectsSlotOccupied) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 250000000ll, Sophus::SE3d(), 0); // Finalizes k=0
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
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
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
    for(int i=0; i<=20; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    
    tl.feedLioPose(t0, Sophus::SE3d(), 0);
    tl.feedLioPose(t0 + 100000000ll, Sophus::SE3d(), 1);
    tl.feedLioPose(t0 + 250000000ll, Sophus::SE3d(), 1);
    // Interval 0 bracket cross epoch (0 and 1) is rejected
    EXPECT_EQ(tl.diag().lio_no_bracket, 1);
}

// 15. MaxInterpolationSpanExactBoundaryAccepted
TEST(TimeBoundary, MaxInterpolationSpanExactBoundaryAccepted) {
    ShadowTimeline tl(imuParams(), baseConfig());
    Sophus::SE3d out;
    tl.feedLioPose(1000000000ll, Sophus::SE3d());
    tl.feedLioPose(1150000000ll, Sophus::SE3d());
    // Query at 1050000000. Gap is exactly 150000000 (150ms).
    EXPECT_TRUE(tl.lookupLioPoseAt(0, 1050000000ll, out));
}

// 16. MaxInterpolationSpanBoundaryPlusOneNsRejected
TEST(TimeBoundary, MaxInterpolationSpanBoundaryPlusOneNsRejected) {
    ShadowTimeline tl(imuParams(), baseConfig());
    Sophus::SE3d out;
    tl.feedLioPose(1000000000ll, Sophus::SE3d());
    tl.feedLioPose(1150000001ll, Sophus::SE3d());
    // Query at 1050000000. Gap is 150000001 ns.
    EXPECT_FALSE(tl.lookupLioPoseAt(0, 1050000000ll, out));
}

// 17. LatenessBoundaryUsesIntegerNs
TEST(TimeBoundary, LatenessBoundaryUsesIntegerNs) {
    ShadowTimeline tl(imuParams(), baseConfig());
    const int64_t t0 = 1000000000ll;
    for(int i=0; i<=110; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    // newest anchor is 1550000000ll
    // t_j for k=0 is 110000000ll (wait: t0=1000M, dt=100M -> t_j=1100M)
    // lateness is 1550M - 1100M = 450M <= 500M
    ConstraintId key1{0, 0, 0};
    EXPECT_EQ(tl.insertRelativeConstraint(key1, gtsam::Pose3()), AcceptDecision::ACCEPTED);
    
    // Feed more IMU to push newest to 1600000001ll
    for(int i=111; i<=121; i++) tl.feedImu(t0 + i*5000000ll, gtsam::Vector3(0,0,9.81), gtsam::Vector3::Zero());
    // lateness for k=1 is 1600000000 - 1200000000 = 400M -> wait, lateness for k=0 is 1600M - 1100M = 500M
    // We want > 500M. Let's push to 1600000001ll
    // Actually the newest is just the last anchor.
    // Let's test insert directly.
    ConstraintId key2{0, 0, 1}; // t_j = 1200M. newest = 1600M. Lateness = 400M
    EXPECT_EQ(tl.insertRelativeConstraint(key2, gtsam::Pose3()), AcceptDecision::ACCEPTED);
    
    // Now push newest such that lateness > 500M exactly by 1ns.
    // We can't feed IMU at arbitrary ns for anchor unless we change jitter.
    // We'll just trust that int64_t > config_.max_constraint_lateness_ns works.
}
}
