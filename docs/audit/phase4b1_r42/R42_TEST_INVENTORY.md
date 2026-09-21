# Phase 4B-1 R4.2 Test Inventory

## Legacy Tests Retained (24 Tests)
1. `Config.DefaultValues`
2. `LioPoseBuffer.DropsBeforeInfluenceHorizonIfFull`
3. `LioPoseBuffer.OutOfOrderFutureSampleDoesNotShadowEndpoint`
4. `LioPoseBuffer.RetainsMinimumSamplesForHorizon`
5. `LioPoseBuffer.JitteredSamplesInterpolatedToAnchorTimes`
6. `SourceEpoch.DifferentEpochSameIdentityAllowed`
7. `SourceEpoch.DelayedOldEpochCannotRollbackCurrentEpoch`
8. `AnchorCreation.GeneratedOnDemandForImuOnly`
... all 24 tests from before.

## New/Updated Tests for R4.2
9. `SourceFinalization.PermutationWithinReorderHorizonCommitsSameMeasurement`: (Updated) Now directly uses `committedMeasurement` oracle to verify the exact measurement inserted matches, regardless of permutation.
10. `SourceFinalization.PermutationWithinReorderHorizonProducesSamePosterior`: (Updated) Now explicitly uses two full timelines with distinct insertion orders, asserting the final posterior solves match.
11. `SourceFinalization.ExactAnchorSampleArrivingLaterBeforeWatermarkWins`: (Updated) Now directly queries `committedMeasurement` instead of the posterior `T`, asserting `EXPECT_NEAR(meas.translation().x(), 0.2, 1e-6)`.
12. `TimeBoundary.LatenessExactBoundaryAccepted`: (New) Tests that lateness precisely `<= max_constraint_lateness_ns` accepts the factor.
13. `TimeBoundary.LatenessBoundaryPlusOneNsRejected`: (New) Tests that lateness `> max_constraint_lateness_ns` by exactly 1ns rejects the factor.
14. `SourceEpoch.CrossEpochBracketGetsExplicitTerminalReason`: (New) Tests that if brackets are across different epochs, it returns `REJECT_CROSS_EPOCH` and updates diagnostics.
15. `SourceFinalization.NotFinalizableAtAnchorEndOnly`: (New) Verifies that intervals do not finalize strictly when watermark passes `t_j`.
16. `SourceFinalization.FinalizableAfterInterpolationInfluenceHorizon`: (New) Verifies that finalization correctly occurs when watermark passes `t_j + max_interpolation_gap_ns`.
17. `SourceFinalization.CloserRightBracketArrivingAfterTjBeforeInfluenceHorizonWins`: (New) Verifies that interpolation inside the influence horizon is successfully integrated before finalization.

### Added in R4.2
- `AnchorGap.GapBreaksFusionSegmentUntilExplicitReset`: Verifies gap segment breaking behavior and failure propagation across gap boundaries.
