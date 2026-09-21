# Phase 4B-2 LIO-only Parity Telemetry Specification

## 1. Objective
The goal of Phase 4B-2 is to achieve deterministic observability into the behavior of the `ShadowTimeline` backend compared to the existing legacy SuperOdom LIO/IMU processing path. This specification defines a read-only telemetry and logging standard to pinpoint the exact interval where shadow and legacy pipelines diverge. 

We will rely on a standardized JSON-lines (`.jsonl`) trace format emitted by both systems to evaluate parity across six functional layers.

## 2. Trace Format Standard
Both Shadow and Legacy systems will emit traces into `shadow_parity_trace.jsonl` and `legacy_parity_trace.jsonl` respectively.

Every line is a valid JSON object with the following top-level schema:

```json
{
  "type": "<EVENT_TYPE>",
  "timestamp_ns": <INT64>,
  "k": <INT32_OR_NULL>,
  "source": "<LIO|IMU|SYSTEM>",
  "epoch": <INT32>,
  "payload": { ... }
}
```

---

## 3. Parity Layers & Payloads

### Layer 1 — Timeline Parity
Tracks the structural lifecycle of intervals and fusion segments.
*   **Event Types**: `TIMELINE_ANCHOR_OPEN`, `TIMELINE_ANCHOR_CLOSE`, `FUSION_SEGMENT_RESET`
*   **Payload Schema**:
    ```json
    {
      "t_k": 1000000000,
      "anchor_status": "GRAPH_INSERTED|SOLVED|INVALID",
      "fusion_segment_status": "ACTIVE|BROKEN"
    }
    ```

### Layer 2 — Input-Consumption Parity
Tracks every sensor measurement submitted to the backend and the outcome (accepted, dropped, gaps).
*   **Event Types**: `INPUT_CONSUME`
*   **Payload Schema**:
    ```json
    {
      "sensor_type": "IMU|LIO",
      "sample_time_ns": 1000005000,
      "action": "ACCEPTED|DROPPED_WATERMARK|DROPPED_LATE|GAP_DETECTED",
      "reason": "late_after_watermark",
      "epoch_changed": false
    }
    ```

### Layer 3 — Factor Parity
Tracks factors physically injected into the `gtsam::NonlinearFactorGraph`.
*   **Event Types**: `FACTOR_INSERT`
*   **Payload Schema**:
    ```json
    {
      "factor_type": "BetweenFactorPose3|ImuFactor",
      "keys": ["X_0", "X_1"],
      "measurement_T_Bi_Bj": [x, y, z, qx, qy, qz, qw],
      "noise_diag": [ ... 6-vector ... ],
      "insertion_time_ns": 1000050000,
      "status": "COMMITTED|FINALIZED"
    }
    ```

### Layer 4 — Gate Parity
Tracks the gating decisions for source constraints relative to the IMU timeline predictions.
*   **Event Types**: `GATE_EVALUATION`
*   **Payload Schema**:
    ```json
    {
      "dT_imu_ref": 0.05,
      "dT_source": 0.05,
      "innovation_6d": [ ... ],
      "translation_norm": 0.01,
      "rotation_norm": 0.001,
      "decision": "ACCEPTED|REJECTED",
      "reason": "TOO_LATE|INNOVATION_TOO_LARGE|SUCCESS",
      "lateness_ns": 15000000,
      "watermark_ns": 900000000,
      "interpolation_brackets": [890000000, 910000000]
    }
    ```

### Layer 5 — State Parity
Snapshots of the solved state variables post-optimization for a given anchor `k`.
*   **Event Types**: `STATE_SNAPSHOT`
*   **Payload Schema**:
    ```json
    {
      "T_W_B": [x, y, z, qx, qy, qz, qw],
      "v_W": [vx, vy, vz],
      "bias_acc": [bx, by, bz],
      "bias_gyro": [bx, by, bz],
      "anchor_status": "SOLVED"
    }
    ```

### Layer 6 — Trajectory Parity
Handled offline using automated evaluation scripts matching `shadow_parity_trace.jsonl` against `legacy_parity_trace.jsonl`.
*   Metrics: Absolute Trajectory Error (ATE), Relative Pose Error (RPE), Time Alignment offset.
*   Goal: Pinpoint the *exact first `k`* where `T_W_B` deviates beyond floating-point threshold, and cross-reference with Layers 1-4 to extract the root cause.

---

## 4. Implementation Rules
1.  **Read-Only**: The telemetry tracer operates solely as a passive observer. It must not alter gating logic, optimizer state, or any R4.2 lifecycle variables.
2.  **No Semantic Changes**: Do not relax lateness checks, watermark rules, or interval interpolation boundaries to artificially create parity.
3.  **JSON Output**: Will use flat, easily parseable string formatting in standard `C++` (avoiding heavy external libraries if not already in tree) mapped directly to a file stream.

## 5. Next Steps
1.  Inject `ParityTracer` hooks into `shadow_timeline.cpp`.
2.  Develop a deterministic synthetic test suite (GTest) that exercises R4.2 lifecycle events and produces a consistent `shadow_trace.jsonl`.
3.  Validate trace consistency via the synthetic suite before proceeding to real-world rosbag tests against the legacy path.
