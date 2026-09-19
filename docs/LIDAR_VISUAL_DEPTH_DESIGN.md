# LiDAR-assisted Visual Depth 设计稿（Phase 3 前置，Gate §19 — 待总设计师 review）

> 状态：DRAFT，供总设计师 review 后才编码。
> 目标：为单目 VIO 的视觉特征提供 LiDAR metric depth 先验，经 `ExternalFeatureDepth`（Gate §15
> 接口，已存在于 `vio_types.hpp`）送入 estimator 边界。**不是再做一套 LVI-SAM**。

## 0. 复用清单（回应"防止又造一套 map/KD-tree"）

| 需求 | 复用 | 不新建 |
|---|---|---|
| 相机投影 | vendored camodocal（与 tracker 同一模型实例） | 不写第二套投影 |
| LiDAR 点输入 | 现有 `featureExtraction`/原始 scan 话题 | 不改 LIO 任何节点 |
| 位姿预测（跨传感器时刻） | 中央 IMU odometry（imuPreintegration 输出）的传播状态 | 不用 VINS 世界系做外参桥 |
| 空间索引 | **逐帧图像域 2D 网格哈希**（每帧重建、O(N)、生命周期=一帧） | 不建全局 3D map/KD-tree/octree；主包 octree 保持 LIO 专用 |
| 深度送入 | wrapper `setExternalDepths()`（已实现，Phase 2c 仅存储） | 不改 VINS vendored 代码 |

## 1. 时间戳与运动补偿

- LiDAR scan 时间戳 `t_L`（VLP-16 10Hz，帧内旋转未补偿——room1 手持慢速，scan 内畸变先按 0 处理，
  记录为已知差异；如需补偿用中央 IMU 传播，不引入新的预积分）。
- 关联时刻 = 相机帧时间戳 `t_C`（测量时间，Gate §13 硬约定）。
- 补偿策略（两档，参数可选）：
  - `identity`：|t_L − t_C| ≤ 50ms，手持场景误差 cm 级，默认档；
  - `imu_propagation`：用中央 IMU odometry 在 [t_L, t_C] 区间的传播位姿做
    `T_W_L(t_L) → T_W_C(t_C)` 相对变换（IMU-centric，与 2021 架构一致）。
- **禁止**用 VINS 世界系（其 gauge 独立于中央系统）作为跨传感器桥，避免 gauge 耦合。

## 2. 帧与方向

```text
p_C = T_C_L · p_L，  T_C_L = T_C_B · T_B_L = (T_B_C)⁻¹ · T_B_L
```

T_B_L 来自 SuperOdom 标定（`imu^R_laser`/`imu^T_laser`），T_B_C 来自 VIO 标定
（kalibr T_cam_imu 求逆，见 FRAMES_AND_CALIBRATION.md §7）。方向与 §2 唯一约定一致，
由测试锁定（非 identity 数值用例）。

## 3. 逐帧深度支撑生成

输入：最近一帧 LiDAR scan（去 NUC 无效点）+ 当前相机帧时间。

1. 每点变换到 C 系：`p_C = T_C_L · p_L`；
2. 距离过滤（`min_range`/`max_range` 参数）；
3. camodocal `spaceToPlane` 投影（z≤0 丢弃）；
4. 像素落在图像内 → 存入 2D 网格（cell ≈ 4px）：cell 内保留 `{depth, point_index}` 列表。

单帧构建成本 O(N_scan)（~3 万点），无全局结构。

## 4. 特征查询与稳健深度

对每个跟踪特征（feature_id, uv）：

1. 在网格中取半径 `pixel_search_radius`（默认 4px）内支撑点集合 S；
2. 拒绝条件（Gate §9.3 逐项）：
   - `|S| < min_support_points`（默认 3）→ 无深度；
   - 深度不连续：`median(depth(S))` 与最远支撑差 > `depth_discontinuity_threshold`
     （默认 0.3m，或相对 10%）→ 拒绝前后景混叠；
   - 支撑深度离散度（MAD）过大 → 拒绝；
3. 深度取 median(S)，不确定性：

```text
sigma = sigma_base + k_range · depth + c_spread · MAD(S)      # 初值 sigma_base=0.02m, k_range=0.01, c_spread=0.5
```

4. 禁止把不确定深度当零噪声真值（Gate §9.4）：ExternalFeatureDepth 携带 sigma 与 support_count。

## 5. 关联生命周期

- 每个 feature_id 维护最近一次关联：`{depth, sigma, stamp_ns, support_count}`；
- `max_lidar_age_sec`（默认 0.2s）超龄 → 标记 invalid，该特征回退纯单目；
- 不做跨帧深度外推（避免与滑窗三角化深度冲突）；VINS 内部对已有关联特征如何消费
  （初始化加速 / inversion / 丢帧回退）Phase 3 编码前按本文件 §7 的边界注入点细化。

## 6. 质量字段与诊断

ExternalFeatureDepth（§15 结构，不变）+ 诊断话题 `vio_depth/health`：
`associated_count / total_features / support_ratio / median_sigma / reject_reasons{no_support, discontinuity, sparse}`。
必须能回答"这一帧为什么该特征没有深度"（任务书 §22 精神）。

## 7. Estimator 注入点（Phase 3 编码时的边界）

- 关联器输出 `vector<ExternalFeatureDepth>` → wrapper `setExternalDepths()`（接口已就位）；
- 消费策略（二选一，编码前定稿）：
  a. 仅用于**三角化初值**（feature_manager 新特征 depth 初值=1/depth），滑窗照常优化；
  b. 作为**深度先验因子**（UnaryFactor on inv_depth，噪声=sigma）加入滑窗。
- 默认 a（对 vendored 代码侵入最小：FeatureManager 已有按 depth 初始化的入口），
  b 视 a 的收益决定是否追加。

## 8. 单元测试计划（Gate §20 对应项）

```text
test_lidar_camera_projection     # 合成 scan + 已知 T_C_L → 投影像素精确
test_depth_association           # 合成场景：特征处有支撑 → depth/sigma 正确
test_depth_discontinuity_rejection  # 前后景混叠 → 拒绝
test_depth_age_expiry            # 超龄 → invalid 回退单目
```

## 9. 明确不做（Phase 3 内）

- 不改 laserMapping/LidarSlam/octree；不做 scan 畸变补偿的完整实现（identity 档下不需要）；
- 不做在线外参标定；不把深度先验当真值（sigma 恒随行）。
