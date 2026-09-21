# Schema Compatibility Matrix (Phase 4B-2A)

This document audits the alignment between the R4.2 Estimator Contract, `tools/parity/schema.json` (the single source of truth), and the Sidecar tools (`validate_trace.py`, `diff_trace.py`, `summarize_trace.py`).

| R4.2 Contract Item | Sidecar Representation (`schema.json`) | Status | Reconciled Action / Rule |
| :--- | :--- | :--- | :--- |
| **Top-Level Envelope** | `schema_version`, `producer`, `type`, `timestamp_ns`, `k`, `source`, `epoch`, `payload` | **ALIGNED** | Strict mode requires all 8 envelope fields. Producer is `"shadow"` or `"legacy"`. `schema_version` is 1. |
| **Anchor/Interval Index $k$** | Top-level `k` integer (0-indexed) or `null` | **ALIGNED** | $k$ represents interval $[t_k, t_{k+1}]$ or anchor $k$. `null` only allowed for non-anchor events (e.g. stream drops, high-rate trajectory). Negative $k$ rejected. |
| **Timestamp Units** | Integer nanoseconds (`int64_t`) | **ALIGNED** | Floats strictly rejected by validator. |
| **SE(3) Pose Coordinate Gauge** | 7-element vector `[x, y, z, qx, qy, qz, qw]` | **ALIGNED** | Quaternion order `qx, qy, qz, qw`. Meters and unit quaternions. Canonicalized. |
| **GTSAM Tangent 6-Vector** | 6-element vector `[rx, ry, rz, tx, ty, tz]` | **ALIGNED** | R4.2 signed order: Rotation(3) first, Translation(3) second. Enforced by tolerance mapping and validator. |
| **Noise Covariance / Sigmas** | 6-element vector `noise_sigmas`: `[rot_sigma_3, trans_sigma_3]` | **ALIGNED** | Diagonal sigmas matching GTSAM tangent order. Field aliases `noise_diag`, `sigmas` permitted. |
| **Anchor Lifecycle Status** | `SCHEDULED`, `GRAPH_INSERTED`, `SOLVED`, `INVALID_GAP` | **ALIGNED** | Monotonic state transitions enforced by `LifecycleTest` in validator. Terminal states (`SOLVED`, `INVALID_GAP`) sticky. |
| **Timeline Intervals & Segments** | `t_k`, `anchor_status`, `interval_status` (`OPEN`/`CLOSED`), `fusion_segment` (`ACTIVE`/`BROKEN`) | **ALIGNED** | `TIMELINE_ANCHOR_OPEN`, `TIMELINE_ANCHOR_CLOSE`, `FUSION_SEGMENT_RESET`. |
| **Input Ingress & Drops** | `sensor_type`, `sample_time_ns`, `action`, `drop_reason`, `epoch_changed`, `coverage_gap` | **ALIGNED** | Actions: `ACCEPTED`, `BUFFERED`, `CONSUMED`, `DROPPED_WATERMARK`, `DROPPED_OUT_OF_ORDER`, `DROPPED_STALE`. Drops explicitly classified. |
| **Relative Factor Committal** | `keys: ["X_k", "X_{k+1}"]`, `measurement_se3`, `noise_sigmas`, `committed`, `finalized` | **ALIGNED** | Factor only emitted upon physical insertion into GTSAM graph. Committed and finalized tracked independently. |
| **Gate Evaluation & Rejection** | `dT_imu_ref` (7-vec), `dT_source` (7-vec), `innovation6` (6-vec), `trans_norm`, `rot_norm`, `thresholds`, `decision`, `reason`, `lateness_ns`, `watermark_ns`, `interpolation_brackets` | **ALIGNED** | Full physical measurements and innovations recorded. Zero placeholders (`"dT_imu_ref": 0`) strictly forbidden. |
| **State Snapshots & Update ID** | `pose` (7-vec), `velocity` (3-vec), `bias_acc` (3-vec), `bias_gyro` (3-vec), `anchor_status: "SOLVED"`, `optimizer_update_id` | **ALIGNED** | Snapshots contain explicit `optimizer_update_id` counter. Only `SOLVED` anchors exposed as solved body states. |
| **First Divergence Semantics** | Chronological interval order ($k$ / time) first; causal layer order at given interval second | **ALIGNED** | Fixed in `diff_trace.py`. Distinguishes `first_chronological_divergence`, `first_causal_layer_at_interval`, and `per_layer`. |
| **Strict Non-Interference** | Read-only observation with zero modification to optimizer graph or logic | **ALIGNED** | Tracing disabled by default. A/B non-interference verified by test suite. |
