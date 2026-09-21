# Legacy Observation Point Map (Phase 4B-2A)

## 1. Executive Summary
This document maps the 6 observation layers of the Phase 4B-2A Parity Trace Schema onto the existing legacy SuperOdom LIO/IMU processing pipeline (`super_odometry` package). 

As required by the taskbook, this mapping:
1. Pinpoints the exact files, functions, variables, frames, and trigger times for legacy telemetry interception.
2. Explicitly analyzes structural and conceptual discrepancies between legacy and shadow implementations without fabricating non-existent 1:1 correspondences.
3. Documents how the single source of truth schema (`tools/parity/schema.json`) captures both architectures.

---

## 2. Legacy Telemetry Observation Point Table

| Parity 层 | Legacy 文件/函数 | 可读取变量 | 坐标系/单位 | 事件时机 | 是否改变行为 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Layer 1: Timeline** | `ImuPreintegration::odometryHandler` (`imuPreintegration.cpp`) | `odomMsg->header.stamp.toSec()`, `key`, `isFirstFrame` | Nanoseconds (`toSec()*1e9`), discrete key counter | 收到 LIO 里程计消息并触发新图节点创建时 | **No** (只读探针) |
| **Layer 2: Input** | `ImuPreintegration::imuHandler` (`imuPreintegration.cpp`) | `imuMsg->header.stamp.toSec()`, `currentAcc`, `currentGyr`, `imuQueOpt` | Body frame (IMU), m/s², rad/s, seconds | IMU 采样压入队列或时间戳倒流丢弃时 | **No** (只读探针) |
| **Layer 2: Input (LIO)** | `ImuPreintegration::odometryHandler` | `laserOdomQueue`, `currentCorrectionTime` | World frame ($W$), seconds | 收到激光点云优化位姿时 | **No** (只读探针) |
| **Layer 3: Factor** | `ImuPreintegration::odometryHandler` | `graphFactors` (`PriorFactor<Pose3>`, `ImuFactor`, `BetweenFactor<ConstantBias>`) | GTSAM keys `X(key)`, `V(key)`, `B(key)` | 因子装配后、调用 `optimizer.update()` 前 | **No** (只读探针) |
| **Layer 4: Gate** | `LaserMapping::scan2MapOptimization` (`laserMapping.cpp`) / `ImuPreintegration::odometryHandler` | `delta_trans`, `delta_rot`, degenerate flags | Translation (m), Rotation (rad) | Scan-to-map 匹配收敛检查与预积分对比时 | **No** (只读探针) |
| **Layer 5: State** | `ImuPreintegration::odometryHandler` | `prevPose_`, `prevVel_`, `prevBias_`, `prevNavState_` | Position (m), Quaternion (xyzw), Vel (m/s), Biases | `optimizer.update()` 优化求解完成并更新状态变量后 | **No** (只读探针) |
| **Layer 6: Trajectory** | `ImuPreintegration::imuHandler` / ROS `/odometry/imu` | `latestEvalState` / `nav_msgs::msg::Odometry` | World frame ($W$), meters, quaternion | 高频 IMU 前向传播输出时或离线关键帧抽取 | **No** (只读离线对拍) |

---

## 3. Structural Conceptual Differences (Legacy vs. Shadow)

### 3.1 Timeline & Anchor Grid
- **Shadow (R4.2)**: Uses an immutable, strictly periodic measurement-time grid $t_k = t_0 + k \times 100\,\text{ms}$ (10 Hz). Intervals are decoupled from laser scan arrival times.
- **Legacy**: Does not have a fixed time-grid anchor timeline. Graph nodes (`key = 0, 1, 2, ...`) are generated dynamically at the timestamps of incoming `laserOdometry` messages (scan rate dependent, typically ~10 Hz with jitter).
- **Parity Representation**: In `schema.json`, both can emit `TIMELINE_ANCHOR_OPEN` / `TIMELINE_ANCHOR_CLOSE`. For legacy, $k$ corresponds to the sequential graph key index, and `t_k` corresponds to the laser scan header timestamp.

### 3.2 Relative Constraint vs. Absolute Prior Factor
- **Shadow (R4.2)**: LIO poses are converted via $T_{B}^L$ into relative between-factors $T_{B_k, B_{k+1}}$ between adjacent anchors ($X_k \to X_{k+1}$). Absolute gauge freedom is anchored strictly once at $t_0$.
- **Legacy**: In `imuPreintegration.cpp` (lines 294-298), legacy adds an absolute `gtsam::PriorFactor<gtsam::Pose3>(X(key), curPose, correctionNoise)`.
- **Parity Representation**: In `schema.json`, `factor_type` distinguishes `"BetweenFactorPose3"` (shadow) from `"PriorFactorPose3"` (legacy). The measurement field `measurement_se3` represents relative pose for shadow and absolute pose for legacy.

### 3.3 Innovation Gating
- **Shadow (R4.2)**: Computes an arrival-order-independent, immutable reference $dT_{\text{imu\_ref}}$ across each anchor interval before external factor influence, and evaluates innovation $e = \text{Logmap}(dT_{\text{imu\_ref}}^{-1} T_{BiBj})$.
- **Legacy**: Does not maintain an immutable pre-factor IMU reference interval trajectory; instead, it checks laser scan matching degeneracy in `laserMapping.cpp` and injects pose priors with dynamic noise inflation into ISAM2.
- **Parity Representation**: `GATE_EVALUATION` records $dT_{\text{imu\_ref}}$ and $dT_{\text{source}}$. In legacy telemetry, if an explicit gate step is injected, it reports the delta between the preintegrated IMU state and the lidar odometry pose.

---

## 4. Status and Implementation Scope
- **Phase 4B-2A Scope**: In this phase, the primary delivery is the hardened, schema-compliant `ShadowTimeline` telemetry and the deterministic synthetic test / sidecar suite.
- **Legacy Telemetry Probe Implementation**: Legacy probe code injection into `super_odometry` is formally scheduled as a follow-up task once the sidecar toolchain and shadow telemetry pass audit review. The static mapping above establishes the schema compatibility and ensures zero architectural barrier for legacy telemetry ingestion.
