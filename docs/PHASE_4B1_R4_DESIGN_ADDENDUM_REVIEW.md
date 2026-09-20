# Phase 4B-1 R4 Evidence Report: ShadowTimeline Lifecycle Convergence

## 1. Overview
We successfully implemented the Phase 4B-1 R4 lifecycle convergence in the `ShadowTimeline` core class. This addresses the 6 reviewer findings related to boundary precision, lifecycle states, latency, IMU coverage gap policies, and identity constraints.

## 2. Implemented Features

### Anchor Lifecycle and Data Structures
* Transformed the `double`-based latency and gap config metrics to `int64_t` nanoseconds to avoid floating-point inclusion errors.
* Introduced `AnchorStatus` (SCHEDULED, OPEN, IMU_COMPLETE, GRAPH_INSERTED, SOLVED, INVALID_GAP).
* Only strictly `SOLVED` anchors are exposed by `anchorState` and `latestAnchor`.

### IMU Gap Policy
* Changed interval closure logic to gracefully mark partial intervals with IMU gaps as `AnchorStatus::INVALID_GAP`.
* If interval $k$ fails to close due to an IMU data gap, the status degrades and propagates forward without attempting to insert unconstrained priors.
* High-rate propagation (`propagateTo`) respects `max_imu_dt_ns` strictly up to the query timestamp. Missing IMU sequences gracefully result in `valid = false`.

### Event-Time Finalization (Watermark)
* Constraint commitments are evaluated strictly against an event-time watermark `max_seen_event_stamp_ns - source_reorder_horizon_ns`.
* This eliminates timestamp inversion constraints completely and ensures arrival-order independence (e.g. out-of-order LIO measurements are buffered and applied safely once the timeline is finalizable).

### Constraint Provenance Separation
* Refactored constraint identification by splitting `ConstraintKey` into `ConstraintId` and `ConstraintSlot`.
* `ConstraintId` deduplicates exact identical replays (`REJECT_DUPLICATE`).
* `ConstraintSlot` handles physical capacity limits across epochs (`REJECT_SLOT_OCCUPIED`).

## 3. Test Coverage & Validation
All 67 tests successfully pass. This includes the 17 specified adversarial regression tests for edge cases:
1. `GapDoesNotCreateValidPlaceholderAnchor`
2. `LatestAnchorIgnoresScheduledUnsolvedAnchor`
3. `AnchorStateRejectsInvalidOrUnsolvedAnchor`
4. `GapIntervalDoesNotInsertMissingKeyFactor`
5. `PropagateToGapReturnsInvalid`
6. `PermutationWithinReorderHorizonCommitsSameMeasurement`
7. `PermutationWithinReorderHorizonProducesSamePosterior`
8. `ExactAnchorSampleArrivingLaterBeforeWatermarkWins`
9. `LateAfterWatermarkDroppedAndCounted`
10. `NoCommitBeforeFinalizable`
11. `SameSourceSameKAcrossEpochRejectsSlotOccupied`
12. `SameIdentityReplayRejectsDuplicateIdentity`
13. `NewEpochFutureIntervalAllowed`
14. `CrossEpochBracketRejected`
15. `MaxInterpolationSpanExactBoundaryAccepted`
16. `MaxInterpolationSpanBoundaryPlusOneNsRejected`
17. `LatenessBoundaryUsesIntegerNs`

A new test runner script `tools/run_phase4b1_r4_tests.sh` was created to support reproducibility in the ROS2 workspace environment.

## 4. Conclusion
The `ShadowTimeline`'s integration logic is now strictly monotonic, rigorously tracked, and gracefully handles gaps and out-of-order arrivals, securing the state machine transitions required for Phase 4B-1.
