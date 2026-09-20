# PHASE 4B-1 Falsification & Hardening Report

**Candidate Baseline Commit:** `4c4006dc63cfcfbf67e3a9d94fc2d4090ea00a2a` (`4c4006d`)  
**Audit Target:** Super Odometry Phase 4B-1 Shadow Central Backend (2021 Fusion Timeline, LIO-only round)  
**Status:** **READY FOR MASTER REVIEW**

---

## 1. Executive Summary

An adversarial contract falsification audit was conducted against candidate commit `4c4006d`. The audit evaluated all signed invariant rules from `docs/FUSION_TIMELINE_DESIGN.md` and Phase 4A gate decisions:
1. Immutable IMU relative prediction reference $dT\_imu\_ref(k, k+1)$ under non-zero initial velocity.
2. Exact measurement-time anchor boundary integration under timestamp jitter.
3. One-shot constraint deduplication with exact $(source, epoch, k)$ key identity.
4. Correct lever-arm normalization under non-zero translation and pure rotation.
5. Single initial gauge priors without recurring injection.
6. Unbounded shadow retention without premature 32-anchor eviction.
7. Arrival-order optimizer invariance of the reference pose.

### Audit Findings Summary
- **Critical Bugs Proven:** 3 signed-contract violations and 1 optimizer estimate refresh bug were mathematically and experimentally proven on candidate `4c4006d`.
- **Hardening Fixes Implemented:** Minimal, non-architectural invariant fixes applied directly to `super_odometry_vio/include/super_odometry_vio/fusion_2021/shadow_timeline.hpp` and `super_odometry_vio/src/fusion_2021/shadow_timeline.cpp`.
- **Test Suite Expansion:** Expanded `test_fusion_shadow` from 10 tests to 18 comprehensive tests.
- **Verification:** 100% green across all 50 workspace test records (44 gtest unit tests).

---

## 2. Proven Bugs & Numerical Evidence

### Bug 1: `dT_imu_ref_[k]` Disregarded Initial Velocity & Gravity Rotation
- **Contract Rule:** "Every anchor interval stores an immutable IMU-only relative prediction $dT\_imu\_ref(k, k+1)$, captured BEFORE any optimization update touches the interval."
- **Candidate Defect (`4c4006d`):**  
  In `shadow_timeline.cpp` line 151:
  ```cpp
  dT_imu_ref_[k] = gtsam::Pose3(pim.deltaRij(), pim.deltaPij());
  ```
  `pim.deltaPij()` evaluates purely $\iint R_t a_t dt^2$. Under non-zero velocity $v_k \neq 0$, the body translation from $B_k$ to $B_{k+1}$ is $R_k^T (p_{k+1} - p_k) = \Delta p_{ij} + R_k^T (v_k \Delta t + \frac{1}{2} g \Delta t^2)$. Candidate `4c4006d` discarded $v_k \Delta t$.
- **Adversarial Audit Proof:**  
  In `TEST(ImuReference, NonZeroVelocityRelativePredictionOracle)`:
  - $v_1 = 0.20\,\text{m/s}$ after interval 0 acceleration.
  - Interval 1 acceleration $a = 2.0\,\text{m/s}^2, \Delta t = 0.1\,\text{s}$.
  - Preintegrated delta position: $\Delta p_{ij} = 0.01000\,\text{m}$.
  - Oracle true relative translation: $0.03000\,\text{m}$.
  - Candidate `4c4006d` stored reference: $0.01000\,\text{m}$.
  - **Discrepancy / Error:** $-0.02000\,\text{m} = -v_1 \Delta t$. (Test FAILED on candidate `4c4006d`).
- **Hardening Fix:**  
  Compute the oracle prediction via `NavState`:
  ```cpp
  const gtsam::NavState prev(anchor_T_W_B_[k], anchor_v_W_[k]);
  const gtsam::NavState predicted = pim.predict(prev, anchor_bias_[k]);
  dT_imu_ref_[k] = anchor_T_W_B_[k].between(predicted.pose());
  ```
- **Post-Fix Result:** Stored reference = $0.03000\,\text{m}$, delta error = $0.00000\,\text{m}$ (PASSED).

---

### Bug 2: Anchor Boundary Endpoint Truncation Under IMU Jitter
- **Contract Rule:** "Exact measurement-time grid $t_k = t_0 + k / \text{anchor\_rate\_hz}$. Continuous, gapless preintegration without invented dt fallbacks."
- **Candidate Defect (`4c4006d`):**  
  In `closeInterval(k)`:
  ```cpp
  if (s.stamp_ns < t_i || s.stamp_ns > t_j) continue;
  ```
  When IMU samples have realistic sensor jitter (e.g. 200 Hz with timestamps straddling $t_j = 0.100\,\text{s}$, e.g. at $0.096\,\text{s}$ and $0.102\,\text{s}$), the sample at $0.102\,\text{s}$ was skipped. Integration ceased at $0.096\,\text{s}$, losing $0.004\,\text{s}$ (4%) of integrated angular velocity and acceleration. Over 1 second, up to $50\,\text{ms}$ was discarded.
- **Adversarial Audit Proof:**
  - **Case A (Rotation):** $\omega_z = 1.0\,\text{rad/s}$ constant yaw rate.
    * Expected yaw over $0.100\,\text{s}$: $0.10000\,\text{rad}$.
    * Candidate pre-fix: $0.09600\,\text{rad}$ ($4.0\%$ error).
    * Post-fix clamped ZOH: $0.10000\,\text{rad}$ (error $< 10^{-6}$).
  - **Case B (Translation/Velocity):** $a_x = 2.0\,\text{m/s}^2$ constant acceleration.
    * Expected velocity: $v_1 = a \Delta t = 0.20000\,\text{m/s}$.
    * Expected translation: $p_1 = \frac{1}{2} a \Delta t^2 = 0.01000\,\text{m}$.
    * Candidate pre-fix: $v_1 = 0.19200\,\text{m/s}, p_1 = 0.00922\,\text{m}$.
    * Post-fix clamped ZOH: $v_1 = 0.20000\,\text{m/s}, p_1 = 0.01000\,\text{m}$ (error $< 10^{-6}$).
- **Hardening Fix:**  
  Under Zero-Order Hold, clamp the boundary at $t_j$:
  ```cpp
  else // s.stamp_ns >= t_j
  {
      const double dt = static_cast<double>(t_j - t_prev) * 1e-9;
      if (dt > 0.0) {
          if (dt <= config_.max_imu_dt_sec) {
              pim.integrateMeasurement(s.acc, s.gyro, dt);
              bracketed = true;
          } else {
              ++diag_.imu_dropped;
          }
      }
      t_prev = t_j;
      break;
  }
  ```

---

### Bug 3: One-Shot Dedup Key Identity & Out-of-Order Factor Skip
- **Contract Rule:** "Source constraints are one-shot immutable; dedup by hash(source, epoch, key_i, key_j); out-of-order arrival supported."
- **Candidate Defect (`4c4006d`):**  
  Candidate stored `std::map<int, uint64_t> inserted_lio_` keyed only by interval index $k$. In `tryInsertLioFactors()`, it set `next_lio_scan_k_ = k + 1` unconditionally upon insertion. If interval 2 arrived before interval 1, interval 1 was permanently skipped. Furthermore, the duplicate check at lines 280-284 was unreachable dead code.
- **Hardening Fix:**
  1. Defined canonical `ConstraintKey`:
     ```cpp
     struct ConstraintKey {
         uint8_t source{0};  // SOURCE_LIO = 0
         uint32_t epoch{0};
         int k{0};
     };
     ```
  2. Maintained `std::set<ConstraintKey> inserted_constraints_` and `std::set<int> lio_constrained_intervals_`.
  3. `next_lio_scan_k_` advances strictly past contiguous constrained intervals, preserving eligibility for delayed bracketing.
  4. Added explicit `insertRelativeConstraint(key, T_Bi_Bj)` with deterministic `AcceptDecision::REJECT_DUPLICATE` diagnostics.

---

### Bug 4: Optimizer Estimates Not Refreshed on LIO Factor Insertion
- **Candidate Defect (`4c4006d`):**  
  In candidate `4c4006d`, `tryInsertLioFactors()` invoked `isam2_.update(g, empty)` directly without setting `graph_dirty_ = true`. When calling `flushOptimizer()`, line 176 `if (!graph_dirty_) return;` aborted immediately, leaving `anchor_T_W_B_` frozen at pre-optimization values.
- **Hardening Fix:**  
  Route all BetweenFactors into `graph_` and assert `graph_dirty_ = true`. `flushOptimizer()` executes the ISAM2 update and refreshes estimates into `anchor_T_W_B_`, `anchor_v_W_`, and `anchor_bias_`, while `dT_imu_ref_` remains strictly untouched.

---

## 3. Invariant Verification Audit

### A. Non-Zero Lever Arm Pure Rotation Invariant
- **Setup:** Body rotated by $\theta = 0.20\,\text{rad}$ in yaw, zero body translation ($p_{WB} = 0$). Lever arm $T_{BL} = (0.25\,\text{m}, 0, 0)$.
- **Raw LiDAR Motion:** $T_{WL_1}.\text{trans}() - T_{WL_0}.\text{trans}() = 0.0499167\,\text{m}$ ($\approx 5\,\text{cm}$ displacement).
- **Normalized Body Relative Pose:** Evaluated via $T_{WB} = T_{WL} \cdot T_{BL}^{-1}$.
- **Result:**
  - Raw displacement norm: $0.04992\,\text{m} > 0.04\,\text{m}$.
  - Optimized body translation norm: $0.00000\,\text{m} < 10^{-4}\,\text{m}$.
  - Innovation errors: 0.

### B. Single Gauge Prior Invariant
- **Setup:** 10.0 seconds of IMU streaming ($100$ intervals at $10\,\text{Hz}$, $2000$ IMU samples).
- **Result:**
  - `diag.pose_priors_added`: **1**
  - `diag.vel_priors_added`: **1**
  - `diag.bias_priors_added`: **1**
  - No recurring source priors added to the graph.

### C. Unbounded Retention Invariant
- **Setup:** 50 intervals closed ($5.0\,\text{s}$).
- **Result:**
  - Total anchors: 51 ($> 32$ anchor boundary).
  - Total graph factors: 103 ($> 100$, no marginalization or variable pruning).
  - All anchors $k \in [0, 50]$ remain retrievable and finite.

### D. Arrival-Order Optimizer Invariance
- **Setup:** Recorded `ref0_before` at interval close. Inserted LIO BetweenFactor that pulled posterior state from $0.00500\,\text{m}$ to $0.00788\,\text{m}$.
- **Result:**
  - `ref0_after.equals(ref0_before, 1e-15)`: **TRUE** (bitwise invariant).
  - Innovation gate evaluation is permanently decoupled from subsequent optimization steps.

---

## 4. Test Suite Accounting

### Workspace Summary
Running `colcon test-result --all --verbose` with `LD_LIBRARY_PATH=/usr/local/lib`:
```text
build/super_odometry/Testing/20260920-0220/Test.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/super_odometry/test_results/super_odometry/test_frames.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/Testing/20260920-0220/Test.xml: 5 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_camera_model.gtest.xml: 3 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_feature_tracker.gtest.xml: 1 test, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_fusion_shadow.gtest.xml: 18 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_lidar_depth.gtest.xml: 12 tests, 0 errors, 0 failures, 0 skipped
build/super_odometry_vio/test_results/super_odometry_vio/test_vins_frames.gtest.xml: 5 tests, 0 errors, 0 failures, 0 skipped

Summary: 50 tests, 0 errors, 0 failures, 0 skipped
```

### Complete Test Catalog (`test_fusion_shadow`: 18 tests)
1. `PoseNoiseOrder.RotationThenTranslationLocked` (GTSAM Pose3 tangent order verification)
2. `AnchorSchedule.ExactMeasurementTimeGrid` (Exact measurement-time grid under jitter)
3. `ImuInterval.OutOfOrderDroppedNotSilentlyFixed` (Out-of-order sample detection)
4. `LioRelativeFactor.StaticRigAcceptedEndToEnd` (Static rig consistency)
5. `LioRelativeFactor.ConstantAccelerationSimulationEndToEnd` (2.0s constant acceleration simulation)
6. `LioRelativeFactor.NonZeroLeverArmPureRotationBodyTranslationZero` (*New adversarial test*)
7. `LioGate.ArrivalOrderIndependence` (Early vs late arrival gate equivalence)
8. `LioGate.TooLateFactorRejected` (Lateness bound rejection)
9. `LioGate.OneShotImmutableNoDoubleInsert` (Replay idempotency)
10. `LioGate.OptimizerPosteriorUpdateDoesNotCorruptImuRef` (*New adversarial test*)
11. `SourceEpoch.ResetDoesNotResetFusionAndBlocksCrossEpoch` (Epoch isolation)
12. `HighRateState.PropagationBetweenAnchors` (Sub-interval propagation)
13. `ImuReference.NonZeroVelocityRelativePredictionOracle` (*New adversarial test*)
14. `AnchorBoundaryIntegration.JitteredSamplesEndpointCaseA_Rotation` (*New adversarial test*)
15. `AnchorBoundaryIntegration.JitteredSamplesEndpointCaseB_TranslationVelocity` (*New adversarial test*)
16. `LioConstraintKey.DuplicateRejectionAndEpochSeparation` (*New adversarial test*)
17. `GaugePrior.ExactlyOnePriorAddedAcrossManyAnchors` (*New adversarial test*)
18. `GraphRetention.UnboundedRetentionBeyond32AnchorsNoEviction` (*New adversarial test*)

---

## 5. Master Review Declaration

All contract falsification criteria have been executed and verified. Proven defects in the candidate implementation `4c4006d` have been resolved with minimal, correct implementations. Workspace test suites are 100% green.

**Declaration:** **READY FOR MASTER REVIEW**
