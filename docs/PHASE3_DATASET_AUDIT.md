# Phase 3 数据集审计（Gate §4）

> 审计日期：2026-09-20 ｜ 原则：不猜缺失外参；缺什么如实记录。

## 1. 数据集盘点

| 数据集 | LiDAR | 相机 | IMU | GT | 结论 |
|---|---|---|---|---|---|
| TUM-VI room1 (ROS2 已转换) | ✗ | ✓ | ✓ | ✓ mocap | 继续**纯 VIO 回归**（depth-off 必须完全等于 Phase 2c 基线） |
| **HILTI22 exp14_basement_2** | ✓ Hesai 10Hz (740 帧/74s) | ✓ 5×Alphasense 40Hz (720×540) | ✓ ~400Hz | ✗ 本地无 GT | **Phase 3 integration 数据集**（§26 明确不要求 ATE 下降，A/B 用内部指标） |
| LVI-SAM handheld (ROS1) | ✓ VLP-16 | ✓ | ✓ | 部分 | 备选，未转换 |
| SuperOdom vlp16_ros2 | ✓ | ✗ | ✓ | ✗ | 无法验证 depth 关联 |

## 2. HILTI22 exp14 明细（实测，非猜测）

- ROS1 bag：`datasets/hilti22/exp14_basement_2.bag`（6.3GB，74.0s），已转换
  `exp14_basement_2_ros2/`（rosbags-convert，标准 sensor_msgs 类型）。
- 话题：`/alphasense/cam0..cam4/image_raw`（每路 2959–2960 帧）、`/hesai/pandar`
  （PointCloud2，740 帧）、`/alphasense/imu`（29539 帧 ≈ 400Hz）。
- ASL 目录（OKVIS2-X 已用它成功运行）无 groundtruth 目录 → GT=无（不影响 §26 验收）。
- `/Point_name`（4 条 String）为序列说明类消息，忽略。

## 3. 标定来源（全部本地已有，零猜测）

来源：`okvis2x_ws/src/OKVIS2-X/config/hilti22/okvis2-lidar.yaml`（HILTI22 官方标定，
OKVIS2-X 在本机用该文件成功跑通同一数据集 → 已验证可用）。

| 项 | 值 | 我们的记法 |
|---|---|---|
| `T_SC`（cam0，OKVIS 语义 p_S = T_SC·p_C） | 4×4 见源文件 | **T_B_C**（方向一致 ✓） |
| cam0 内参 | equidistant k1..k4 = [-0.036462, -0.005445, 0.002754, -0.001109]；f=350.373/350.464；c=(367.598, 253.848)；720×540 | camodocal `KANNALA_BRANDT`（k2..k5=k1..k4，mu/mv/u0/v0） |
| `T_SL`（lidar 节，"IMU {S} ↔ LiDAR {L}"，p_S = T_SL·p_L） | [0,-1,0,-0.0007; -1,0,0,-0.0086; 0,0,-1,0.0550; 0,0,0,1] | **T_B_L**（方向一致 ✓） |
| `T_BS` | I | body=IMU ✓ |
| 相机间外参 | T_SC cam1..cam4 | 仅用 cam0（VINS 单目） |
| **T_C_L** | T_C_L = inverse(T_B_C) · T_B_L（Gate §6 公式） | 运行时计算 + 单测锁定 |

## 4. HILTI22 专项注意

- Hesai PointCloud2 字段含 per-point timestamp（Hesai 驱动 `timestamp` 字段，float 秒）→
  设计文档 §temporal 采用"有点级时间"档；实现按"字段存在才用"处理，缺失则回退 scan 时间戳 + max_age（Gate §10）。
- 图像 720×540 与 TUM-VI 512×512 不同 → 相机模型/图像尺寸全部参数化，无硬编码。
- IMU 400Hz：预积分 dt≈2.5ms，无需改核心。

## 5. A/B 与回退评测计划（无 GT 场景）

- A/B（§25）：同一 VINS 参数，`lidar_depth.enable` false/true 各跑 exp14；比较 initialization
  time、failed triangulation 数、invalid/negative depth 数、feature lifetime、runtime p50/p95。
- LiDAR dropout（§28）：bag 回放时对 `/hesai/pandar` 按 [t0+30s, t0+45s) 停止投喂（注入节点或
  采样脚本），验证 VIO 无缝退回普通 VINS、无 NaN、无约束中断。
- 投影正确性：debug overlay（§21）人工肉眼检查 + 合成场景单测。
