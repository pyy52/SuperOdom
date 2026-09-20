# PHASE 4B-1 Falsification & Hardening Report

**Baseline Commit:** `7a9a943c30947d771b5cb8fc7d2ba28e14350ce2`  
**Candidate Baseline Commit (in tree):** `4c4006d587e804b91e15414b4119f10350bbb85a`  
**Superseded Planning SHA:** `4c4006dc63cfcfbf67e3a9d94fc2d4090ea00a2a` (superseded placeholder in early task notes; actual git object is `4c4006d587e804b91e15414b4119f10350bbb85a`)  
**Audit Target:** Super Odometry Phase 4B-1 Shadow Central Backend (2021 Fusion Timeline, LIO-only round)  
**Status:** **READY FOR MASTER REVIEW**

---

## 1. Commit Provenance & Test Accounting Disambiguation

To preserve rigorous audit reproducibility, the commits and their respective validation records are strictly partitioned:

| Milestone / State | 40-Character Commit SHA | Compare URL (vs Baseline) | Test Suite Accounting |
|---|---|---|---|
| **Phase 4A Baseline** | `7a9a943c30947d771b5cb8fc7d2ba28e14350ce2` | Baseline origin | N/A (Design doc only) |
| **Phase 4B-1 Candidate** | `4c4006d587e804b91e15414b4119f10350bbb85a` | [7a9a943...4c4006d](https://github.com/pyy52/SuperOdom/compare/7a9a943c30947d771b5cb8fc7d2ba28e14350ce2...4c4006d587e804b91e15414b4119f10350bbb85a) | **41 test records** (10 shadow fusion tests, 31 other) |
| **Phase 4B-1 Hardened (R2)** | `70617e5896b6c5533107f8d712d043ff9caee48d` | [7a9a943...70617e5](https://github.com/pyy52/SuperOdom/compare/7a9a943c30947d771b5cb8fc7d2ba28e14350ce2...70617e5896b6c5533107f8d712d043ff9caee48d) | **50 test records** (18 shadow fusion tests, 32 other) |
| **Phase 4B-1 Hardened (R3 / Final)** | HEAD | Compare URL on GitHub | **56 test records** (24 shadow fusion tests, 32 other) |

> [!NOTE]
> Early task notes referenced `4c4006dc63cfcfbf67e3a9d94fc2d4090ea00a2a`. The actual git object created in the repository and pushed to GitHub is `4c4006d587e804b91e15414b4119f10350bbb85a` (both share prefix `4c4006d`). All audits and diffs operate strictly on the genuine git object `4c4006d587e804b91e15414b4119f10350bbb85a`.

---

## 2. Executive Summary

An adversarial contract falsification audit and two rounds of independent reviewer source audits were conducted against the Phase 4B-1 shadow backend. All signed invariant rules from `docs/FUSION_TIMELINE_DESIGN.md` and Phase 4A gate decisions were audited:
1. Immutable IMU relative prediction reference $dT\_imu\_ref(k, k+1)$ under non-zero initial velocity (A1).
2. Exact measurement-time anchor boundary integration under timestamp jitter with right-sample ZOH policy (A2).
3. Anchor-time source pose interpolation (translation LERP + rotation SLERP) and arrival-order independent bracketing (A3/A4/B-01/B-03).
4. Monotonic source epoch handling and stale sample rejection (A4/B-02).
5. Measurement-time high-rate propagation with binary-search historical anchor selection (B-04).
6. Continuous interval IMU coverage tracking without partial-gap closures (B-05).
7. One-shot constraint deduplication with exact $(source, epoch, k)$ key identity (A4).
8. Lever-arm normalization under non-zero translation and pure rotation (A3).
9. Single initial gauge priors without recurring injection (A7).
10. Unbounded shadow retention without premature 32-anchor eviction (A7).
11. Arrival-order optimizer invariance of the reference pose (A5).

---

## 3. Review Findings & Mathematical / Architectural Resolution

### B-01: Arrival-Order Independent LIO Endpoint Selection
- **Defect:** In candidate and initial hardened code, right endpoint search used `if (s.stamp_ns >= t_j && !s_j) s_j = &s;`, which selected the first-arrived sample after $t_j$. A future sample arriving before the true endpoint permanently shadowed the correct endpoint.
- **Resolution:** Replaced raw sample search with `lookupLioPoseAt(epoch, stamp_ns, T_W_L)` which finds $s_{before}$ (maximum stamp $\le t$) and $s_{after}$ (minimum stamp $\ge t$), guaranteeing arrival-order independence regardless of buffer insertion order.
- **Discriminating Test:** `LioPoseBuffer.OutOfOrderFutureSampleDoesNotShadowEndpoint` feeds $t_0$, then future sample $t_0 + 0.30\text{s}$ (with large $2.0\text{m}$ offset), then true endpoint $t_0 + 0.10\text{s}$. Verifies interval 0 uses $t_0 + 0.10\text{s}$ and passes the innovation gate.

### B-02: Monotonic Source Epoch & Stale Sample Dropping
- **Defect:** In `feedLioPose`, `current_lio_epoch_ = lio_epoch;` was assigned unconditionally. A delayed sample from an old epoch rolled `current_lio_epoch_` backward.
- **Resolution:** Enforced monotonic epoch updates:
  ```cpp
  if (lio_epoch < current_lio_epoch_) {
      ++diag_.lio_stale_skipped;
      return;
  }
  if (lio_epoch > current_lio_epoch_) {
      current_lio_epoch_ = lio_epoch;
  }
  ```
- **Discriminating Test:** `SourceEpoch.DelayedOldEpochCannotRollbackCurrentEpoch` feeds epoch 0, advances to epoch 1, then feeds delayed epoch 0. Verifies `lio_stale_skipped` increments, epoch does not roll back, and no stale factor enters the graph.

### B-03: Source Pose Interpolation at Anchor Timestamps
- **Defect:** The implementation previously picked raw bracketing samples rather than evaluating source pose at exact anchor times $t_i, t_j$ as signed in `FUSION_TIMELINE_DESIGN.md:87-95`.
- **Resolution:** Implemented `lookupLioPoseAt(epoch, stamp_ns, T_W_L)`:
  - Exact match shortcut when $s.\text{stamp\_ns} == \text{stamp\_ns}$.
  - Double-sided bracketing within `config_.max_interpolation_gap_sec`.
  - Normalized fraction $\alpha = \frac{t - t_{before}}{t_{after} - t_{before}}$.
  - Translation linear interpolation: $p = (1-\alpha) p_{before} + \alpha p_{after}$.
  - Rotation spherical linear interpolation: $q = q_{before}.\text{slerp}(\alpha, q_{after})$.
  - `tryInsertLioFactors()` interpolates at both $t_i$ and $t_j$.
- **Discriminating Test:** `LioPoseBuffer.JitteredSamplesInterpolatedToAnchorTimes` feeds jittered samples ($-0.02\text{s}, +0.03\text{s}, +0.08\text{s}, +0.13\text{s}$) around anchors $0.0\text{s}$ and $0.10\text{s}$. Verifies exact interpolated anchor poses ($0.0\text{m}$ and $0.20\text{m}$, $0.05\text{rad}$) and factor acceptance.

### B-04: High-Rate State Historical Propagation
- **Defect:** `propagateTo(stamp_ns)` hardcoded anchor index $k = \text{anchor\_stamps\_}.\text{size}() - 1$. Querying a historical timestamp (e.g. $t_0 + 0.55\text{s}$ when anchors exist up to $t_0 + 1.0\text{s}$) integrated from anchor 10 instead of anchor 5.
- **Resolution:** Implemented binary search anchor selection:
  ```cpp
  auto it = std::upper_bound(anchor_stamps_.begin(), anchor_stamps_.end(), stamp_ns);
  if (it == anchor_stamps_.begin()) return out; // invalid: timestamp precedes first anchor
  const int k = static_cast<int>((it - anchor_stamps_.begin()) - 1);
  ```
  Integrates IMU forward from $\text{anchor\_stamps\_}[k]$ to `stamp_ns`.
- **Discriminating Test:** `HighRateState.PropagateToHistoricalDynamicTimestampUsesCorrectAnchor` simulates constant acceleration $a_x = 2.0\text{ m/s}^2$ across 10 intervals ($1.0\text{s}$). Queries $t_0 + 0.55\text{s}$ and verifies returned state matches analytic values ($v_x = 1.10\text{ m/s}, x = 0.3025\text{ m}$) rather than anchor 10 values ($x \ge 1.0\text{ m}$).

### B-05: A2 Boundary Policy & Full-Coverage Tracking
- **Defect:** `closeInterval(k)` used a single boolean `bracketed` that allowed intervals with invalid internal gaps ($dt > \text{max\_imu\_dt\_sec}$) to close with incomplete integration. Also, boundary sample ownership under ZOH was not test-locked.
- **Resolution:**
  - Implemented continuous interval coverage tracking: `covered_until_ns` starts at $t_i$. Any invalid subsegment sets `gap_detected = true`. Interval closure strictly requires `!gap_detected && covered_until_ns == t_j`.
  - Documented and locked the **Right-Sample ZOH policy**: each subsegment $[t_{prev}, t_{next}]$ uses the right endpoint measurement.
- **Discriminating Tests:**
  - `AnchorBoundaryIntegration.NonConstantBoundarySampleOwnership`: feeds step input $\omega_z(0.098\text{s}) = 1.0\text{ rad/s}, \omega_z(0.103\text{s}) = 3.0\text{ rad/s}$ across boundary $t_1 = 0.100\text{s}$. Verifies total yaw is exactly $0.104\text{ rad}$, strictly falsifying left-sample ZOH ($0.100\text{ rad}$).
  - `AnchorBoundaryIntegration.InvalidInternalGapDoesNotCloseInterval`: injects an invalid internal gap ($0.050\text{s} > \text{max\_dt } 0.02\text{s}$) followed by boundary sample. Verifies interval is NOT closed, `intervals_closed == 0`, `imu_dropped > 0`, and `totalGraphFactors() == 0`.

### B-06: Provenance Documentation Cleanup
- **Defect:** Package documentation had conflicting candidate SHA references.
- **Resolution:** Clarified that `4c4006dc63cfcfbf67e3a9d94fc2d4090ea00a2a` was an early planning placeholder, superseded by actual git SHA `4c4006d587e804b91e15414b4119f10350bbb85a`. Cleaned all reports and rebuilt all patch files.

---

## 4. Test Suite Accounting

### Workspace Summary
Running `colcon test-result --all --verbose` inside Docker `superodom-ros2:latest`:
```text
build/super_odometry/Testing/20260920-0318/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/super_odometry/test_results/super_odometry/test_frames.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/Testing/20260920-0318/Test.xml: 5 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_camera_model.gtest.xml: 3 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_feature_tracker.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_fusion_shadow.gtest.xml: 24 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_lidar_depth.gtest.xml: 12 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_vins_frames.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped

Summary: 56 tests, 0 errors, 0 failures, 0 skipped
```

### Complete Test Catalog (`test_fusion_shadow`: 24 tests)
1. `PoseNoiseOrder.RotationThenTranslationLocked` (GTSAM Pose3 tangent order verification)
2. `AnchorSchedule.ExactMeasurementTimeGrid` (Exact measurement-time grid under jitter)
3. `ImuInterval.OutOfOrderDroppedNotSilentlyFixed` (Out-of-order sample detection)
4. `LioRelativeFactor.StaticRigAcceptedEndToEnd` (Static rig consistency)
5. `LioRelativeFactor.ConstantAccelerationSimulationEndToEnd` (2.0s constant acceleration simulation)
6. `LioRelativeFactor.NonZeroLeverArmPureRotationBodyTranslationZero` (Lever-arm normalization under pure rotation)
7. `LioGate.ArrivalOrderIndependence` (Early vs late arrival gate equivalence)
8. `LioGate.TooLateFactorRejected` (Lateness bound rejection)
9. `LioGate.OneShotImmutableNoDoubleInsert` (Replay idempotency)
10. `LioGate.OptimizerPosteriorUpdateDoesNotCorruptImuRef` (Arrival-order optimizer invariance)
11. `SourceEpoch.ResetDoesNotResetFusionAndBlocksCrossEpoch` (Epoch isolation)
12. `SourceEpoch.DelayedOldEpochCannotRollbackCurrentEpoch` (*New R3 adversarial test — B-02*)
13. `HighRateState.PropagationBetweenAnchors` (Sub-interval propagation)
14. `HighRateState.PropagateToHistoricalDynamicTimestampUsesCorrectAnchor` (*New R3 adversarial test — B-04*)
15. `ImuReference.NonZeroVelocityRelativePredictionOracle` (Non-zero velocity IMU reference oracle — A1)
16. `AnchorBoundaryIntegration.JitteredSamplesEndpointCaseA_Rotation` (Boundary integration rotation — A2)
17. `AnchorBoundaryIntegration.JitteredSamplesEndpointCaseB_TranslationVelocity` (Boundary integration translation — A2)
18. `AnchorBoundaryIntegration.NonConstantBoundarySampleOwnership` (*New R3 adversarial test — B-05 right-sample ZOH*)
19. `AnchorBoundaryIntegration.InvalidInternalGapDoesNotCloseInterval` (*New R3 adversarial test — B-05 coverage tracking*)
20. `LioConstraintKey.DuplicateRejectionAndEpochSeparation` (One-shot key deduplication)
21. `GaugePrior.ExactlyOnePriorAddedAcrossManyAnchors` (Gauge prior counter verification)
22. `GraphRetention.UnboundedRetentionBeyond32AnchorsNoEviction` (Unbounded retention, exact 103 factor check)
23. `LioPoseBuffer.OutOfOrderFutureSampleDoesNotShadowEndpoint` (*New R3 adversarial test — B-01 arrival order*)
24. `LioPoseBuffer.JitteredSamplesInterpolatedToAnchorTimes` (*New R3 adversarial test — B-03 source interpolation*)

---

## 5. Master Review Declaration

All contract falsification criteria and all Round 3 review findings (B-01 through B-06) have been resolved and verified with discriminating unit tests. No architectural changes or Design Change Proposals were required. The full workspace test suite is 100% green (56/56 tests passing).

**Declaration:** **READY FOR MASTER REVIEW**
