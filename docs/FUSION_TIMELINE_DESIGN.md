# FUSION_TIMELINE_DESIGN — 2021 中央 IMU 融合时间线设计（Phase 4A 交付）

> 依据：`AGENT_PHASE4A_FUSION_TIMELINE_DESIGN.md`（Gate 全文已读）。
> 本文件**只做设计**；设计评审通过前不改中央图任何代码（Gate §57）。
> 基线：Phase 3 = PASS，commit `439b9dd`。

## 1. 组件与数据流总图

```text
                     raw IMU (200–400Hz, measurement time)
                            │  feedImu()
                            ▼
              ┌──────────────────────────────┐
              │   FusionTimeline (2021 mode)  │
              │  AnchorScheduler（IMU 时间轴主动建锚）│
              │  ImuIntervalBuffer            │
              │  SourcePoseBuffer(LIO/VIO)    │
              │  RelativeConstraintBuilder    │
              │  ConstraintGate               │
              │  FusionBackend2021 (GTSAM)    │
              │  HighRateStatePropagator      │
              └──────────┬───────────────────┘
                         │ /fusion_2021/shadow_state（body 系）
                         ▼
   legacy imuPreintegration.cpp 原样运行 ──→ /integrated_to_init（现有输出不动）
   laserMapping.cpp（LIO）─────────────────── 原样运行
   super_odometry_vio（VIO）───────────────── 原样运行
```

模式边界（§42）：

```yaml
fusion:
  mode: legacy_lio_prior     # 默认；现有行为逐位保持
  # mode: imu_centric_2021   # shadow backend 联调期手工切换
```

## 2. 锚点时间线（§5/§6/§7）

```text
IMU 测量时间轴 ────────────────────────────────────────→
   t0        t1        t2        t3        t4      （10Hz 锚点）
   │─────────│─────────│─────────│─────────│
  X0,V0,B0  X1,V1,B1  X2,V2,B2  X3,V3,B3  X4,V4,B4
   ├──IMU──┤├──IMU──┤├──IMU──┤├──IMU──┤
LIO 相对因子 ΔT_LIO(0→1)（可迟到）
VIO 相对因子 ΔT_VIO(2→3)（可迟到）
```

- **锚点由 IMU 流在测量时间轴上主动创建**（t_now 越过 t_k 即建 X_k/V_k/B_k 并立即插入
  IMU preintegration factor + bias BetweenFactor），LiDAR/VIO callback 永不建节点。
- 锚点率 `anchor_rate_hz: 10.0`（= 最低频 odometry 频率，§6；运行期固定，不随源健康度变化）。
- 因子按测量时间戳落在锚点区间，端点 key 查表；无源约束时纯 IMU 链照常存在。

## 3. 迟到因子（§17/§18/§19/§20）

```text
测量时间：  t9.9 ────── t10.0(锚点已建) ────── t10.1
到达时间：        LIO 因子(0→10) 于 ICP 35ms 后到达
                  VIO 因子(9→10) 于 Ceres 18ms 后到达
处理：  端点 key ∈ retained keys？ lateness ≤ bound？ 未重复？
        → 全部满足才插入（key 寻址，与到达顺序无关）
        → 否则 REJECT_TOO_LATE / REJECT_DUPLICATE + 诊断计数
```

- `max_constraint_lateness_sec: 0.5`（初值，待标定）；
  `retained_anchor_count: 32`（= 3.2s @10Hz > max lateness × 安全系数 6，满足
  retention horizon > max_constraint_lateness，§41）。
- 因子身份 = `constraint_id = hash(source, source_epoch, key_i, key_j)`（§19 去重）。
- 乱序到达天然支持：VIO(k+1) 先于 LIO(k) 到达时各自按 key 挂靠（§20，含专项合成测试）。

## 4. 源 epoch（§11/§12/§13/§39/§40）

```text
VIO clearState()/reboot/重初始化 → vio_epoch++（wrapper 已拥有该状态机，加计数器）
LIO：审计现有 mapping reset 路径；当前 slim 版正常建图不改 gauge → lio_epoch 恒 0
      直到真实 reset 出现（不得由退化/健康度伪造）
中央自身 reset（灾难性 bias/优化器失败/非有限状态）→ fusion_epoch++，
      清空锚点表与 pending factors（§40）
拒绝规则：epoch(t_i) ≠ epoch(t_j) → REJECT_CROSS_EPOCH；源 epoch ≠ 中央当前 epoch → 同
```

**源重启不重启中央图**：VIO reboot 只是 `vio_epoch++` + 暂时无 VIO 因子，中央靠 IMU+LIO 继续。

## 5. 源位姿适配与相对约束构造（§10/§14/§15/§47）

```cpp
struct SourcePoseSample {
  int64_t stamp_ns;  uint32_t epoch_id;
  Sophus::SE3d T_sourceWorld_B;   // 源任意世界系，中央不关心
  SourceHealth health;  SourceQuality quality;
};
// 适配器求锚点时刻源位姿（同 epoch 且有双侧包围样本，平移线性插值 + SO(3) slerp，
// 最大插值跨度 0.15s（初值）；否则 NO_SOURCE_CONSTRAINT——非中央故障）
// ΔT_ij = inverse(T_sourceWorld_B(t_i)) · T_sourceWorld_B(t_j)   ← 源 gauge 消去
```

类型化约束接口（§47，**不再往 covariance[] 塞魔法 flag**）：

```text
fusion_2021/RelativeConstraint.msg:
  uint8 SOURCE_LIO=0 / SOURCE_VIO=1
  uint32 source_epoch_id
  int64 stamp_i_ns, stamp_j_ns
  float64[7] T_Bi_Bj            # 平移+四元数(x,y,z,w)，GTSAM BetweenFactor 语义
  uint8 health                  # 0 normal / 1 degraded / 2 bypass
  float32 quality_scale         # 有界 [1, max_scale]
  float32[2] innovation_hint    # 可选
  uint64 constraint_id
```

方向约定（§9）：`T_Bi_Bj = inverse(X_i)·X_j`（body 相对位姿），
`test_gtsam_between_factor_body_relative_direction` 非 identity 单测锁定。

## 6. IMU 区间切片（§21/§22）

`ImuIntervalBuffer`：按测量时间戳为每个 [t_i, t_j] 提取样本；严格正 dt、乱序/重复时间戳
检出并丢弃+计数、端点 bracketing、零阶保持策略确定化。**2021 路径不保留 legacy 的
`dt=0.005` 静默兜底**（legacy 路径原样保留兼容）。预积分数学一律 GTSAM
`PreintegratedImuMeasurements`/`ImuFactor`——新代码只拥有切片/缓冲/边界处理。

## 7. 初始 gauge 与先验（§23/§24/§43）

- X0：position=0、yaw=0、roll/pitch 由 IMU 重力对齐；V0/B0 用现有 IMU 初始化 + 一次性
  PriorFactor(X0/V0/B0)。**一次性 gauge 先验 ≠ legacy 的逐次 LiDAR 绝对 PriorFactor**（§24）。
- `imu_centric_2021` 模式下**禁止**同时加 legacy LiDAR 绝对先验与新相对因子（双重计数，§43）；
  legacy 模式下 shadow 只输出不注入，天然不双计。

## 8. 可靠性 → 权重（§25–§31）

`ConstraintGate` 返回 ACCEPT_NORMAL / ACCEPT_DOWNWEIGHTED / REJECT(+原因)。

- VIO 噪声 = 名义 σ_rot/σ_trans × `quality_scale`，`quality_scale ∈ [1.0, max_scale=8.0]`
  （初值；由 inlier 数、reprojection、visual_observability_proxy、solver health 组合的
  确定性映射，**不得**把 H proxy 当协方差，§26）。
- LIO 复用现有 degeneracy/alignment-risk/fitness（`LidarSlam` 特征值分析），映射为
  reject/weight scale，不造第二套退化检测（§27）。
- 健康映射（§30）：ACTIVE→normal；DEGRADED→downweighted；BYPASS/RECOVERING/WARMUP→无因子；
  非有限 SE3→立即 REJECT_INVALID_SE3。DEGRADED 的 scale 不得超过 ACTIVE（§28）。
- 鲁棒核：Huber（可配置，文档化；不替代健康门控，§29）。
- **LiDAR depth 覆盖率不直接进入中央权重**（§31）——中央权重只看 VIO 最终估计器质量。
- Innovation gate（§49/§50）：平移 [m] 与旋转 [rad] 分开阈值（初值 1.0m / 0.35rad，待标定），
  结合源健康/退化/IMU 年龄决策；粗大离群→REJECT_INNOVATION，中度分歧→downweight。

## 9. 因果双向耦合（§32/§33/§34）

```text
对 t 时刻源帧：
  1. 中央按 IMU 传播到 t（不含该帧新约束）
  2. 预测交给 VIO/LIO 作 prior/初值
  3. 源优化
  4. 源发出相对约束
  5. 中央稍后吸收
—— 同一测量的预测不得已包含其自身约束（无即时自反馈）
```

- 中央→VIO：VIO 维护中央高率预测短缓冲，按**相机测量时间戳**插值/传播取用（禁止"最新
  callback 状态"），body 系输入、相机外参 VIO 内部处理（§33）。
- 中央→LIO：保持现有 T_B_L 变换思路不动 LIO 接口（§34）。

## 10. 状态输出与高率重传播（§35/§36/§37/§38）

- 内部真状态 = `T_W_B`；新接口 `/fusion_2021/body_state`（T_W_B, v_W, b_a, b_g,
  fusion_epoch, health, stamp）；legacy `/integrated_to_init` 保持现有 LiDAR 系兼容输出
  （适配器 `T_W_L = T_W_B · T_B_L`），中央图不优化 LiDAR 系状态。
- 每次优化更新后：最新锚点态 + 当前 bias + 锚点后缓冲 IMU → 高率传播当前态；
  迟到因子修正锚点 → 重算锚点 + 重新传播 → 发布修正后的高率态。
- **发布时间戳单调不回退**；历史修正走诊断话题。

## 11. Legacy→Shadow→切换（§44/§45）

```text
Phase 4B-1 shadow backend（独立话题输出，legacy 继续驱动一切）
→ 4B-2 LIO-only parity（对照 legacy）
→ 4B-3 VIO-only + epoch 测试
→ 4B-4 LIO+VIO
→ 4B-5 双向预测闭环
```

不整体 fork imuPreintegration.cpp；抽取复用 IMU 初始化/标定/参数/帧变换/高率发布适配器，
新时间线实现为聚焦库（`FusionTimeline/AnchorScheduler/ImuIntervalBuffer/SourcePoseBuffer/
RelativeConstraintBuilder/ConstraintGate/FusionBackend2021/HighRateStatePropagator`，
命名仅示意）。

## 12. 诊断与测试

- 拒绝原因枚举（§48）：ACCEPTED/DOWNWEIGHTED/REJECT_SOURCE_BYPASS/REJECT_CROSS_EPOCH/
  REJECT_INVALID_SE3/REJECT_TOO_LATE/REJECT_ENDPOINT_MISSING/REJECT_INTERPOLATION_GAP/
  REJECT_DUPLICATE/REJECT_INNOVATION/REJECT_SOURCE_QUALITY——全部带计数诊断。
- §51 合成时间线测试 21 项全部落位；§52 无 ROS 的确定性因子图仿真（匀速+定常 yaw rate+已知
  LIO/VIO 相对位姿+注入延迟/乱序/VIO reset/LIO 退化段，终态须在数值容差内一致）。
- §53 数据集验证顺序：VLP+IMU（legacy vs 2021 LIO 隔离对比）→ TUM-VI（VIO-only+epoch）→
  最后多模态。

## 13. 决策表（§55，无架构性 TBD）

| 问题 | 决策 |
|---|---|
| 中央锚点时钟 | 测量时间（IMU 流时间戳），锚点由 IMU 流主动创建 |
| 默认锚点率 | `anchor_rate_hz: 10.0`（运行期固定） |
| 锚点创建触发 | IMU 测量时间越过锚点时刻即创建并插入 IMU 因子 |
| IMU 区间边界策略 | ImuIntervalBuffer：正 dt、乱序/重复丢弃+计数、端点 bracketing、零阶保持；无静默 dt 兜底 |
| LIO 位姿查询 | SourcePoseBuffer 双侧包围样本插值（平移线性+SO(3) slerp） |
| VIO 位姿查询 | 同上；按锚点测量时间查询，epoch 一致才可用 |
| 最大插值跨度 | 0.15s（初值，待标定） |
| 最大迟到因子时延 | 0.5s（初值，待标定） |
| 图保留范围 | retained_anchor_count=32（3.2s > max lateness×6） |
| 源 epoch 表示 | uint32 epoch_id，源内部 clearState/reinit 时递增（wrapper 已有状态机挂钩） |
| 中央 epoch 表示 | fusion_epoch_id，中央 reset 时递增并作废旧 pending 约束 |
| VIO DEGRADED 权重 | quality_scale 降档（×2，初值）；ACTIVE=×1；有界 ≤ max_scale=8 |
| LIO 退化权重 | 复用 LidarSlam 特征值退化 → REJECT/downweight 映射 |
| Innovation 阈值 | 平移 1.0m / 旋转 0.35rad 分立（初值，待标定） |
| Legacy 兼容模式 | `fusion.mode: legacy_lio_prior`（默认）/ `imu_centric_2021` |
| Shadow 后端话题 | `/fusion_2021/shadow_state` |
| 最终状态帧接口 | 内部 body_state（T_W_B…）；legacy 传感器系话题保留适配层 |

## 14. 现有代码映射（§56，不重复造轮子）

| 所需能力 | 现有本地代码 | 复用/适配/新建 |
|---|---|---|
| IMU 初始化 | `imuPreintegration.cpp` reset_graph/failureDetected 逻辑 | 复用（shadow 抽取共享） |
| IMU 标定/噪声参数 | `config/*.yaml` + `parameter.cpp` | 复用 |
| 高率 IMU 队列 | `imuPreintegration.cpp`（MapRingBuffer 类似物）+ VIO `feedImu` 环 | 复用+适配 |
| GTSAM 优化器 | `imuPreintegration.cpp`（iSAM2/ISAM2 参数） | 复用（新实例） |
| LIO 位姿输出 | `laserMapping.cpp` `pubLaserOdometry` | 复用（订阅端适配） |
| LIO 退化 | `LidarSlam.cpp` 协方差特征值 + Pos/Ori 阈值 | 复用（直接接入 gate） |
| IMU→LIO 预测 | `laserMapping.cpp` `selectPosePrediction`（LIO_ODOM/IMU 档） | 复用 |
| VIO 位姿输出 | wrapper `latestState/RelativePose` | 复用 |
| VIO epoch/reset | wrapper 状态机（BYPASS/RECOVERING 已有） | 适配（加 epoch 计数） |
| VIO 质量 | wrapper VioQuality（visual_observability_proxy 等） | 复用（gate 输入） |
| body/sensor 外参 | `parameter.cpp` + `FRAMES_AND_CALIBRATION.md` | 复用 |
| 相对约束类型消息 | （新建）`fusion_2021/RelativeConstraint.msg` | 新建（§47） |
| Anchor/timeline 后端 | （新建）`FusionTimeline` 库 | 新建（聚焦库，不 fork 700 行节点） |

## 15. 测试与验收挂钩

- 设计评审通过后才进入 4B-1；4B 各阶段验收以 §51/§52 测试全绿 + 对应 parity 指标为准。
- 风险声明：reboot 高发数据集（如 HILTI22 exp14）下 2021 模式的优势预期体现为
  valid-output 时间与分段连续性提升（Phase 3 B1 已示范该现象），而非全局 ATE。
