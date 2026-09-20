# LiDAR-assisted Visual Depth 设计（Phase 3，依 AGENT_PHASE3_LIDAR_VISUAL_DEPTH.md 修订）

> 状态：v2，按总设计师 Phase 3 任务书 §3 清单逐项定义；随附 `PHASE3_DATASET_AUDIT.md`。
> 硬约束：中央 GTSAM 封锁；无第二套 map/KD-tree/octree；projection 只用 camodocal；
> depth ≡ camera 光轴 Z_C；vendor 文件零改动（预期）。
> 原则：`reuse > adapt > extend > rewrite`。

## 1. 话题与数据流（integration 数据集 = HILTI22 exp14，见数据集审计）

```text
/hesai/pandar (PointCloud2, 10Hz)  ──┐
/alphasense/imu (Imu, ~400Hz)      ──┤→ LidarDepthAssociator → ExternalFeatureDepth[] → VioEstimator
tracked features (PointCloud,10-20Hz)┘        (边界模块，禁止进 KLT)
```

- TUM-VI room1：继续纯 VIO 回归数据（无 LiDAR → 关联器自然空转，验证 fallback）。

## 2. 时间戳定义（全部 measurement time，Gate §20）

| 量 | 定义 |
|---|---|
| `t_img` | 图像 header stamp（VINS 特征帧时间） |
| `t_scan` | LiDAR scan header stamp |
| `t_pt`（可选） | Hesai PointCloud2 `timestamp` 字段（float 秒，per-point）；字段缺失则退化为 t_scan |

## 3. Deskew / motion compensation（§9/§10）

现状审计：SuperOdom 现有 LIO 未对 VLP-16/Hesai 做点级 deskew（`featureExtraction` 直接按帧处理）；
无可复用的 point-time helper → 本阶段不引入完整 deskew（§2 禁止为写而写）。

采用两档 causal 策略：

- **scan-stamp 档（默认）**：整 scan 视作 t_scan 采集，`max_age_sec` 严格门控（默认 0.12s）；
  不假装整 scan 同时采集——超过 max_age 的 scan 直接不入支撑池。
- **point-time 档（Hesai 有 t_pt 时）**：对参与关联的点用 t_pt 做同一变换链下的相对时间修正
  （用 IMU 传播位姿 T_W_B(t) 内插）。**Phase 3 实现顺序：先 scan-stamp 档跑通并量化 age 分布，
  point-time 档仅在 age 分布证明必要时启用**（参数 `time_mode`）。

不依赖未来数据（无 unbounded future-data dependency）；多 scan 积累按 §8 bounded + causal。

## 4. 帧与外参（HILTI22 值见数据集审计 §3，来源 OKVIS2-X 官方标定文件）

```text
T_B_L = T_SL（OKVIS lidar 节，方向一致：p_B = T_B_L · p_L）
T_B_C = T_SC（cam0，p_B = T_B_C · p_C）
T_C_L = inverse(T_B_C) · T_B_L        # Gate §6 公式，单测锁定
p_C = T_C_L · p_L
```

测试：`test_lidar_camera_extrinsic_composition`、`test_lidar_point_to_camera_non_identity`（非 identity 数值）。

## 5. 深度数学定义（§5 锁死）

VINS 特征使用归一化光线 `[x, y, 1]`，故 external depth 必须是

```text
depth ≡ Z_C = (T_C_L · p_L).z()      # camera 光轴深度
```

**不是** LiDAR range `‖p_L‖`，也不是 `‖P_C‖`。测试 `test_depth_semantic_z_not_range`：
构造 45° 斜视点，断言输出 = Z_C 且 ≠ range/‖P_C‖。

## 6. Projection（§7 不造第二套）

```text
P_L → P_C = T_C_L · p_L → camodocal::Camera::spaceToPlane(P_C) → (u, v)
```

模型实例与 tracker 使用同一 camodocal 工厂加载（HILTI22: KANNALA_BRANDT, 720×540）。
禁止任何 `fx*x/z+cx` 直写路径。测试：`test_lidar_projection_non_identity`、
`test_lidar_projection_fisheye_model`（EquidistantCamera 实例）。

## 7. Support cloud（§8/§12：bounded deque + 图像域桶，非 map）

```text
class LidarDepthBuffer {                      # bounded
  std::deque<RecentLidarScan> scans_;         # max_scans（参数，初值 3）/ max_age_sec / max_points
  void push(RecentLidarScan&&);               # 超界丢最老
};
```

`RecentLidarScan`：`{stamp_ns, std::vector<Eigen::Vector4d> xyz_B, t_pt 可选}`（B 系，T_B_L 已应用）。

每图像帧构建**临时图像域索引**（生命周期=本帧，栈上对象）：

```text
for scan in scans_ (age ≤ max_age):
  for p in scan (age ≤ max_age):
    p_C = T_C_L · p_L（scan-stamp 档：T_C_L 按 t_scan 与 t_img 的相对位姿修正可选）
    if p_C.z() ≤ z_min / ≥ z_max → reject (BEHIND_CAMERA / OUT_OF_RANGE)
    (u,v) = camera.spaceToPlane(p_C) → FOV 内 → bucket[(u/cell, v/cell)].push({u,v,Z_C,P_C,t_pt,t_scan})
cell ≈ 4px；桶即用即弃 → 无任何全局 3D 结构。
```

## 8. 关联算法（§13/§14：robust median，禁止朴素平均）

对每个 tracked feature (id, u_f, v_f)：

1. 收集半径 `pixel_search_radius`（默认 4px）内桶候选；
2. 按 Z_C 升序排序；
3. 取中位数层：以中位 Z_C 为中心，保留 |Z−median| ≤ `max_depth_spread_m` 的内层簇
   （前景/背景混叠时只保留有足够 support 的近层——保守）；
4. 稳健校验：MAD > `max_relative_depth_spread · median` → HIGH_SPREAD 拒绝；
5. `support_count < min_support_points`（默认 3）→ NO_SUPPORT 拒绝；
6. 输出 `ExternalFeatureDepth{feature_id, image_stamp_ns, depth_z_m=median, robust_sigma_m,
   support_count, median_pixel_distance, lidar_age_sec, valid}`。

**拒绝原因枚举（§24）**：NO_SUPPORT / TOO_OLD / BEHIND_CAMERA / OUTSIDE_FOV / DEPTH_DISCONTINUITY /
HIGH_SPREAD / INVALID_NUMERIC——全部计数入诊断。
错误 depth 比没有 depth 更危险：任何一项不过 → 拒绝，绝不平均。

## 9. External depth 注入点与生命周期（§16/§17/§19）

边界（禁止进 KLT）：

```text
FeatureTracker → tracked features → LidarDepthAssociator → VioEstimator(wrapper) → FeatureManager
```

注入机制（**vendor 零改动**，全部走 vendored 公共 API）：

```cpp
// processImage 返回后（本帧三角化已完成），对目标特征做 seed/repair：
VectorXd d = f_manager.getDepthVector();
// 定位目标特征（solve_flag==0 且 estimated_depth<=0 → §17A 初始化；
//              estimated_depth<=0/negative → §17B 修复）
d[idx] = 1.0 / Z_C;      # inverse-depth state 的初值
f_manager.setDepth(d);
```

- 生效路径：下一次 `vector2double()` 将 estimated_depth 写入 para_Feature → 滑窗优化以此为初值。
- **§17C 成熟特征默认不覆盖**：仅触碰 solve_flag==0 / depth 无效特征；已三角化特征只做
  一致性诊断（|depth_lidar − depth_vins| 统计，不写入）。
- **§19 生命周期**：depth 以 `(feature_id, image_stamp_ns)` 键绑定（ExternalFeatureDepth 内嵌
  image_stamp_ns）；消费时核对特征观测帧时间，禁止把未来帧深度塞给旧帧。

## 10. Fallback（§28）

- `lidar_depth.enable=false`：关联器不构造、零开销 → 必须完全等于 Phase 2c 基线（回归项）。
- LiDAR dropout：buffer 空/超龄 → 关联器输出空集，VINS 退回普通单目行为，无任何 mandatory 依赖。
- 与 Phase 2c blackout 逻辑正交：无 feature → 无 depth 关联，BYPASS 行为不变。

## 11. Debug overlay（§21 硬要求）

关联器节点输出 `vio_depth/overlay`（sensor_msgs/Image，`debug_overlay` 参数，默认 false）：
底图为当前帧 → 画 LiDAR 投影点（灰）、accepted 特征（绿 + 深度数字）、rejected 特征（红 + 原因首字母）、
tracked 特征（蓝）。另输出可选 `vio_depth/fov_cloud`（相机 FOV 内 LiDAR 云）。
验收 = 人工肉眼核对投影对齐（保存若干帧 PNG 存档）。

## 12. Runtime budget（§30，逐段 p50/p95）

| 段 | 目标（6 核容器） |
|---|---|
| LiDAR preprocessing（变换+过滤） | < 2ms p95 |
| projection + bucketing | < 3ms p95 / 帧 |
| association（~150 特征） | < 1ms p95 |
| total depth-stage | < 5ms p95（20Hz 帧预算 50ms 的 10%） |

实现为纯栈/预分配结构；scan cache 上限 = max_scans × max_points 参数化。

## 13. Threading（§29）

LiDAR 回调只入 bounded deque（max_scans），图像 worker 消费——图像回调永不等待 LiDAR；
VIO 实时优先于"必须找到 depth"。

## 14. 配置（§34，沿用参数系统）

```yaml
lidar_depth:
  enable: false                # benchmark 完成前默认 false
  camera_config_file: ...      # camodocal yaml（与 tracker 同格式）
  lidar_topic: /hesai/pandar
  T_B_L / T_B_C: 来自标定文件（不硬编码）
  time_mode: scan_stamp        # point_time 为可选档
  max_scans: 3
  max_age_sec: 0.12
  min_depth_m: 0.3
  max_depth_m: 30.0
  pixel_search_radius: 4.0
  min_support_points: 3
  max_depth_spread_m: 0.3
  max_relative_depth_spread: 0.10
  debug_overlay: false
```

## 15. 测试计划（§22/§23 全清单对应）

§22 列表 16 项逐条落位（core 纯单测 + wrapper 级 seed/repair/no-overwrite 测试走
FeatureManager 公共 API 的合成帧用例）；§23 运动补偿合成用例（已知 body 运动/点时/相机时/外参 →
解析验证 P_C/pixel/Z_C）。`test_depth_disabled_vins_regression` = TUM-VI depth-off 全程回归。

## 16. visual_observability_proxy 边带任务（§31）

Phase 3 内完成：inlier mask（只统计观测对）、pixel noise 权重（σ_px=1.0 常数起步）、
rot(rad)/trans(m) 分别归一化输出三组指标（normalized 6DOF / translation-only / rotation-only）。
仍不称 central fusion covariance。

## 17. Provenance（§32）

LVI-SAM 仅作思路 reference（image-space depth association 概念）；不搬代码。
THIRD_PARTY.md 增补记录：inspected files（depth 关联思路）、repo/commit、license。
