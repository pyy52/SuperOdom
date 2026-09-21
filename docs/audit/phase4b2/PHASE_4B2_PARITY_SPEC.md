# Phase 4B-2A LIO-only Parity Telemetry Specification

## 1. Objective
The goal of Phase 4B-2A is to establish deterministic, machine-readable, schema-valid observability into the behavior of the `ShadowTimeline` backend compared to the existing legacy SuperOdom LIO/IMU processing path. This specification defines a read-only telemetry and logging standard to pinpoint the exact interval, causal layer, and field where shadow and legacy pipelines diverge, identifying the root cause without altering frozen R4.2 estimator semantics or lifecycle logic.

---

## 2. Trace Format Envelope (Schema Version 1)
Both Shadow and Legacy systems emit traces into `.jsonl` files (e.g. `shadow_parity_trace.jsonl` and `legacy_parity_trace.jsonl`). Every line is a single, valid, self-contained JSON object matching the single source of truth defined in `tools/parity/schema.json`.

Top-level schema envelope:
```json
{
  "schema_version": 1,
  "producer": "shadow",
  "type": "<EVENT_TYPE>",
  "timestamp_ns": 1000000000,
  "k": 0,
  "source": "IMU|LIO|SYSTEM",
  "epoch": 0,
  "payload": { ... }
}
```

### Envelope Field Constraints
- `schema_version` (`int`): Currently `1`. Required on every record.
- `producer` (`str`): Either `"shadow"` or `"legacy"`. Required on every record.
- `type` (`str`): One of the canonical event types listed in Section 3.
- `timestamp_ns` (`int64`): Nanosecond integer timestamp. Must never be float.
- `k` (`int` or `null`): 0-indexed anchor/interval index. `null` only for events not associated with an anchor/interval.
- `source` (`str`): Sensor or subsystem origin (`"IMU"`, `"LIO"`, `"SYSTEM"`).
- `epoch` (`int`): Source sensor epoch (tracks resets/jumps; non-decreasing per stream).
- `payload` (`dict`): Layer-specific payload object.

---

## 3. Parity Layers & Canonical Payloads

### SE(3) and Tangent Coordinate Conventions (Locked)
- Quaternion order: `[qx, qy, qz, qw]` in `[x, y, z, qx, qy, qz, qw]` (7-element vector). Units: meters.
- GTSAM tangent / innovation: 6-vector `[rx, ry, rz, tx, ty, tz]` (Rotation 3 first, Translation 3 second).
- Noise sigmas: 6-vector `[rot_sigma_3, trans_sigma_3]` matching GTSAM tangent order.

---

### Layer 1 — Timeline Parity
Tracks the structural lifecycle of timeline anchors and fusion segments.
* **Event Types**:
  - `TIMELINE_ANCHOR_OPEN`
  - `TIMELINE_ANCHOR_CLOSE` / `TIMELINE_INTERVAL_CLOSE`
  - `FUSION_SEGMENT_RESET`
* **Canonical Payload Schema**:
  ```json
  {
    "t_k": 1000000000,
    "anchor_status": "SCHEDULED|GRAPH_INSERTED|SOLVED|INVALID_GAP",
    "interval_status": "OPEN|CLOSED",
    "fusion_segment": "ACTIVE|BROKEN"
  }
  ```
* **Payload Field Aliases**:
  - `fusion_segment_status` -> `fusion_segment`
  - `interval_stamp_ns`, `anchor_stamp_ns` -> `t_k`

---

### Layer 2 — Input-Consumption Parity
Tracks raw sensor samples received, buffer ingress, and admission/drop decisions.
* **Event Types**:
  - `INPUT_CONSUME`
  - `INPUT_SAMPLE`
  - `INPUT_DROP`
* **Canonical Payload Schema**:
  ```json
  {
    "sensor_type": "IMU|LIO",
    "sample_time_ns": 1000005000,
    "action": "ACCEPTED|BUFFERED|CONSUMED|DROPPED_WATERMARK|DROPPED_OUT_OF_ORDER|DROPPED_STALE",
    "drop_reason": "none|late_after_watermark|stale_skipped|out_of_order",
    "epoch_changed": false,
    "lateness_ns": 0,
    "coverage_gap": false
  }
  ```
* **Payload Field Aliases**:
  - `reason` -> `drop_reason`
  - `imu_stamp_ns`, `lio_stamp_ns` -> `sample_time_ns`

---

### Layer 3 — Factor Parity
Tracks factors physically committed and inserted into the optimization factor graph.
* **Event Types**:
  - `FACTOR_INSERT`
  - `FACTOR_FINALIZE`
* **Canonical Payload Schema**:
  ```json
  {
    "factor_type": "BetweenFactorPose3",
    "keys": ["X_0", "X_1"],
    "measurement_se3": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "noise_sigmas": [0.01, 0.01, 0.01, 0.05, 0.05, 0.05],
    "committed": true,
    "finalized": true,
    "insertion_time_ns": 1000050000
  }
  ```
* **Payload Field Aliases**:
  - `measurement_T_Bi_Bj`, `measurement` -> `measurement_se3`
  - `noise_diag`, `sigmas`, `sigma6` -> `noise_sigmas`
* **Rules**:
  - `measurement_se3` must be a 7-element double array `[x, y, z, qx, qy, qz, qw]`.
  - `noise_sigmas` must be a 6-element double array `[rot3, trans3]`.

---

### Layer 4 — Gate Parity
Tracks source constraint acceptance/rejection decisions relative to the immutable IMU reference prediction.
* **Event Types**:
  - `GATE_EVALUATION`
  - `GATE_DECISION`
* **Canonical Payload Schema**:
  ```json
  {
    "dT_imu_ref": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "dT_source": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "innovation6": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "trans_norm": 0.01,
    "rot_norm": 0.001,
    "thresholds": [0.5, 0.2],
    "decision": "ACCEPTED|REJECTED",
    "reason": "SUCCESS|TOO_LATE|INNOVATION_TOO_LARGE_TRANS|INNOVATION_TOO_LARGE_ROT|NO_BRACKET|CROSS_EPOCH|DUPLICATE|SLOT_OCCUPIED",
    "lateness_ns": 15000000,
    "watermark_ns": 900000000,
    "left_bracket_ns": 890000000,
    "right_bracket_ns": 910000000,
    "interpolation_brackets": [890000000, 910000000]
  }
  ```
* **Payload Field Aliases**:
  - `translation_norm` -> `trans_norm`
  - `rotation_norm` -> `rot_norm`
  - `innovation_6d`, `innovation_6` -> `innovation6`
* **Zero Placeholder Prohibition**:
  - `dT_imu_ref` and `dT_source` must be genuine 7-element SE(3) poses. Never scalar zeros or dummy values.
  - `innovation6` must be the genuine 6-element tangent error vector `[rot3, trans3]`.
  - `interpolation_brackets` must record genuine sample timestamps.

---

### Layer 5 — State Parity
Snapshots of solved body state variables post-optimization for a given anchor `k`.
* **Event Types**:
  - `STATE_SNAPSHOT`
  - `STATE_UPDATE`
* **Canonical Payload Schema**:
  ```json
  {
    "pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    "velocity": [0.0, 0.0, 0.0],
    "bias_acc": [0.0, 0.0, 0.0],
    "bias_gyro": [0.0, 0.0, 0.0],
    "anchor_status": "SOLVED",
    "optimizer_update_id": 1
  }
  ```
* **Payload Field Aliases**:
  - `T_W_B` -> `pose`
  - `v_W` -> `velocity`
* **Optimizer Identity**:
  - Each optimization flush advances `optimizer_update_id`. Snapshots emitted per update retain explicit revision identity.

---

### Layer 6 — Trajectory Parity (Offline)
Evaluated offline by toolchain comparing shadow and legacy trajectories extracted from Layer 5 `SOLVED` snapshots.
- No SE(3) gauge alignment or manual time shifts are applied in strict parity mode.
- Direct interval-by-interval pose comparison using `diff_trace.py`.

---

## 4. First-Divergence Semantics
When comparing two parity traces:
1. **Chronological Interval Order First**: Divergences are ordered by interval index `k` (or timestamp if `k` is null). An earlier interval divergence always precedes a later interval divergence.
2. **Causal Layer Order at Given Interval**: If multiple layers diverge at the same interval `k`, the divergence in the earliest causal layer (`timeline` < `input` < `factor` < `gate` < `state` < `trajectory`) is reported as the causal root.
3. **Per-Layer Summaries**: The toolchain retains and reports the earliest divergence within each layer independently.

---

## 5. Implementation Rules
1. **Read-Only Non-Interference**: Telemetry operations must never modify solver graph, estimator state, decision variables, or timing semantics. Tracing disabled vs enabled must produce strictly identical estimator outputs.
2. **Deterministic Output**: JSON output uses fixed C-locale formatting with `std::numeric_limits<double>::max_digits10` precision. Non-finite values (`NaN`, `Inf`) are strictly rejected.
3. **Fail-Safe File Handling**: Tracing is disabled when file path is empty. If a path is provided but cannot be opened, an explicit diagnostic error is raised.
