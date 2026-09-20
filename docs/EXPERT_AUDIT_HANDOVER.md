# SuperOdom ROS 2 Shadow Timeline — Phase 4B-1 Independent Audit Handover

Dear reviewer,

This package is provided for independent audit of the ROS 2 migration and reconstruction of the SuperOdom multi-sensor fusion backend, specifically the Phase 4B-1 **LIO-only Shadow Timeline**.

The goal of this phase is not to maximize benchmark performance. The goal is to verify the semantic correctness of:

```text
factor graph state definition
IMU preintegration reference construction
measurement timestamp anchoring
LIO/body frame normalization
source epoch isolation
one-shot adjacent factor insertion
backend retention lifecycle
```

---

## 1. Source target and evidence package

### Source target

```text
Repository: https://github.com/pyy52/SuperOdom
Branch: ros2
Source target commit: de6a3b0c63293442aca85ed3170f54e86f0e5104
```

### Evidence package

```text
Release: https://github.com/pyy52/SuperOdom/releases/tag/v4b1-audit-r3-evidence-v2
Evidence ZIP: SuperOdom_Phase4B1_Core_de6a3b0_evidence_v2.zip
Evidence ZIP SHA-256: be6d157e14a883e35d7d05971ef4736e0b54d90fc7ee6d4cf28ee4440dec6926
```

The evidence package is an external audit artifact generated from the clean source target commit above. It may contain logs and generated patches that are not part of the source commit itself.

---

## 2. Provenance chain

```text
Phase 4A baseline:
7a9a943c30947d771b5cb8fc7d2ba28e14350ce2

Phase 4B-1 candidate:
4c4006d587e804b91e15414b4119f10350bbb85a

Round 2 hardened:
70617e5896b6c5533107f8d712d043ff9caee48d

Round 3 audit target:
de6a3b0c63293442aca85ed3170f54e86f0e5104
```

The early string:

```text
4c4006dc63cfcfbf67e3a9d94fc2d4090ea00a2a
```

is a superseded planning placeholder and must not be used as a candidate commit.

---

## 3. Core files for review

```text
docs/FUSION_TIMELINE_DESIGN.md
docs/PHASE_4B1_FALSIFICATION_REPORT.md
docs/PHASE_4B1_R3_CLOSURE_EVIDENCE.md
super_odometry_vio/include/super_odometry_vio/fusion_2021/shadow_timeline.hpp
super_odometry_vio/src/fusion_2021/shadow_timeline.cpp
super_odometry_vio/test/test_fusion_shadow.cpp
super_odometry_vio/CMakeLists.txt
```

Primary implementation functions to inspect:

```text
lookupLioPoseAt()
closeInterval()
insertRelativeConstraint()
propagateTo()
```

---

## 4. Signed Phase 4B-1 contract

Please audit against the following contract:

```text
central state: T_W_B
anchor grid: t_k = t0 + k * 0.1s
immutable pre-fusion ΔT_imu_ref(k,k+1)
all source gates use that immutable reference
LIO T_W_L -> body frame before relative differencing
GTSAM Pose3 tangent/noise order: [rotation(3), translation(3)]
retention: unbounded_shadow
external factor span: adjacent X_k -> X_{k+1} only
constraint identity: one-shot (source, epoch, k)
source epoch reset does not reset central graph
Phase 4B-1: LIO only
no silent dt=0.005 fallback
gauge prior once only
```

The anchor grid is defined by **measurement timestamps**, not message arrival order.

---

## 5. Key audit questions

### Q1 — IMU reference semantics

Please verify that the immutable gate reference is not built as:

```cpp
Pose3(pim.deltaRij(), pim.deltaPij())
```

The R3 implementation claims to construct it via GTSAM state prediction:

```cpp
predicted_j = pim.predict(anchor_state_i, anchor_bias_i);
dT_imu_ref = pose_i.between(predicted_j.pose());
```

Audit whether:

```text
initial velocity contribution is included
gravity/state prediction is included
bias used is the correct anchor/PIM bias
reference is captured before external factors for the interval
reference is never recomputed from optimized posterior states
```

---

### Q2 — LIO anchor-time interpolation and frame conversion

Please verify the claimed implementation sequence:

```text
1. lookup/interpolate T_W_L(t_i) at anchor timestamp t_i
2. lookup/interpolate T_W_L(t_j) at anchor timestamp t_j
3. convert each absolute pose to body:
   T_W_B(t) = T_W_L(t) * inverse(T_B_L)
4. build body relative measurement:
   T_Bi_Bj = between(T_W_B(t_i), T_W_B(t_j))
5. insert an adjacent factor X_i -> X_j only when j = i+1
```

Please specifically test:

```text
jittered LIO timestamps around anchors
out-of-order LIO arrival
exact anchor sample priority
duplicate timestamps
non-zero lever arm + body rotation
```

---

### Q3 — source epoch and one-shot causality

Please verify:

```text
current_lio_epoch_ never moves backward
late old-epoch samples cannot insert factors
old buffered samples cannot bracket new-epoch lookups
central graph is not reset by source epoch changes
one-shot identity is exactly (source, epoch, k)
```

Please also check whether rejected factor attempts consume one-shot identity or whether only accepted constraints do. If the design is ambiguous, flag it.

---

### Q4 — IMU interval boundary ownership

The R3 implementation claims a **Right-Sample ZOH** policy:

```text
each subsegment [t_prev, t_next] uses the IMU sample at t_next
terminal boundary segment [t_prev, t_j] uses the first sample with stamp >= t_j
```

Please verify:

```text
sum of integrated dt equals t_j - t_i
continuous coverage is required before interval close
gaps > max_imu_dt_sec prevent interval closure
duplicate/out-of-order IMU timestamps do not create silent fallback
no dt=0.005 is introduced
```

---

### Q5 — high-rate propagation

Please verify that `propagateTo(stamp_ns)` selects the retained anchor:

```text
t_k <= stamp_ns < t_{k+1}
```

and propagates forward from that anchor, rather than always starting from the latest anchor.

Test dynamic motion, not only static IMU.

---

### Q6 — Pose3 noise / covariance

Please verify the GTSAM Pose3 ordering:

```text
[rotation(3), translation(3)]
```

If the current implementation uses fixed Pose3 sigmas rather than SE(3) adjoint covariance propagation from source covariance, please classify whether this is acceptable for Phase 4B-1 or should be deferred to a later adapter phase.

---

### Q7 — future VIO extensibility

Please assess whether the current Phase 4B-1 structures can later support VIO without changing signed 4B-1 semantics:

```text
multi-source constraint keys
source epoch isolation
source-specific interpolation
adjacent factor insertion
gate reference immutability
telemetry needed for first-divergence debugging
```

Do not require VIO implementation in Phase 4B-1.

---

## 6. Reported tests

The current R3 claim is:

```text
workspace tests: 56
shadow fusion tests: 24
errors: 0
failures: 0
skipped: 0
```

Please verify against:

```text
r3_build.log
r3_test.log
test_fusion_shadow raw output
```

Do not rely on the summary alone.

---

## 7. Known non-blocking risk

The core unit-test backend still defaults `T_B_L` to identity for synthetic tests.

For live LIO integration, unconfigured extrinsic and explicitly configured identity extrinsic must be distinguishable. This is a Phase 4B-2 live-adapter requirement, not a Phase 4B-1 core blocker if clearly documented.

---

## 8. Reproduction note

If a project-provided Docker image is available:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
colcon build --packages-select super_odometry super_odometry_vio
colcon test --packages-select super_odometry super_odometry_vio
colcon test-result --all --verbose
/root/ros2_ws/build/super_odometry_vio/test_fusion_shadow
```

If `superodom-ros2:latest` is a local/internal image, please state that explicitly and provide Dockerfile or image-build instructions before claiming external one-click reproducibility.

---

## 9. Requested review output

Please return findings in this format:

```text
Finding
Why it matters
Evidence
Signed rule affected
Minimal correction
Required local verification
```

Recommended status options:

```text
NO BLOCKER FOUND
BLOCKER FOUND
CONTRACT AMBIGUITY
INFRA / REPRODUCIBILITY ISSUE
```

Please do not declare Phase PASS. The final phase gate remains with the Master Designer after independent review.
