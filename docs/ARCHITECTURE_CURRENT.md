# SuperOdom 当前架构（Phase 1 交付）

> 对应 commit：`f10e65cd`（ros2 分支）。描述的是**当前实际代码**，不是论文目标架构。
> 目标架构与差距见 `IMPLEMENTATION_AUDIT.md` §4。

## 1. 节点与数据流（现状）

```text
                       /imu/data (sensor_msgs/Imu)
                            │
                            v
                 ┌──────────────────────┐
 /velodyne_points│ imu_preintegration   │  最终输出 nav_msgs/Odometry
        │        │ (GTSAM 中央图)        ├──────────────→ integrated_to_init  ←系统最终状态
        v        │  Prior(P/V/B)        │
 ┌─────────────┐ │  ImuFactor           │──→ ~/imu_odom (nav_msgs/Odometry) ──┐
 │ feature_    │ │  Between(bias)       │──→ ~/imu_path                       │
 │ extraction  │ │  + LIO 位姿校正 ←────┼──────────────────────────────┐      │
 │ (VLP-16特征) │ └──────────────────────┘                              │      │
 └──────┬──────┘                                                        │      │
        │ ~/feature_info (LaserFeature)                                 │      │
        v                                                               │      │
 ┌──────────────────────────────────────────────────┐                   │      │
 │ laser_mapping_node                                │                  │      │
 │  octree/LocalMap 地图                              │                  │      │
 │  Ceres 解析边/面残差 + Tukey + ScaledLoss           │                  │      │
 │  DENSE_SVD 协方差 → 退化判定(Pos/Ori 阈值)          │                  │      │
 │  setInitialGuess(): 五源预测切换                    │ ←─────────────────┘      │
 │   {LIO_ODOM, VIO_ODOM(空槽), NEURAL_IMU_ODOM(空槽), │                          │
 │    IMU_ORIENTATION, CONSTANT_VELOCITY}             │                          │
 └──────┬────────────────────────────────────────────┘                          │
        │ ~/laser_odometry (nav_msgs/Odometry) → 作为 LIO→IMU 校正进入中央图 ────┘
        │ ~/laser_odometry_incremental
        │ ~/vio_prediction, ~/lio_prediction, ~/prediction_source   ← 已发布、无生产者
        └──→ /health(Bool), ~/optimization_stats
```

要点：

1. **IMU 中心已成立**：最终状态（`integrated_to_init`）由 `imu_preintegration_node` 的 GTSAM 图输出，不是 LIO。
2. **LIO↔IMU 双向闭环已存在**：IMU odometry 作为 LIO 初值（`setInitialGuess`），LIO 结果回灌中央图校正
   （`process_imu_odometry`）。这正是论文架构的一半，任务书 §6.1 判断正确：**不改动**。
3. **VIO 三个接口位全部空缺**：视觉特征跟踪、sliding-window VIO、VIO→中央图约束。
   其中 LIO 侧的"消费 VIO 预测"槽位（`PredictionSource::VIO_ODOM` + `vio_prediction_status`）上游已留好。

## 2. 话题清单（当前实际使用）

| 话题 | 类型 | 方向 | 生产者/消费者 |
|---|---|---|---|
| `/imu/data` | sensor_msgs/Imu | 订阅 | imu_preintegration、laserMapping（IMU 预测） |
| `/velodyne_points` | sensor_msgs/PointCloud2 | 订阅 | feature_extraction |
| `~/feature_info` | super_odometry_msgs/LaserFeature | 发布→订阅 | feature_extraction → laser_mapping |
| `~/laser_odometry` | nav_msgs/Odometry | 发布→订阅 | laser_mapping → imu_preintegration（LIO 校正） |
| `~/laser_odometry_incremental` | nav_msgs/Odometry | 发布 | laser_mapping |
| `integrated_to_init` | nav_msgs/Odometry | **发布（最终状态）** | imu_preintegration |
| `~/imu_odom`、`~/imu_path` | Odometry/Path | 发布 | imu_preintegration |
| `~/vio_prediction`、`~/lio_prediction` | nav_msgs/Odometry | 发布（无生产者填充） | laser_mapping |
| `~/prediction_source` | String | 发布 | laser_mapping |
| health / stats | Bool / OptimizationStats / IterationStats | 发布 | imu_preintegration、laser_mapping |

（前缀 `~` 为各节点命名空间，实际以 `PROJECT_NAME` 参数为准。）

## 3. 坐标系与外参约定（速览）

- 世界系 `map`（可旋转副本 `map_rot`），传感器系 `sensor`；中央状态位姿 = `T_W_B`（IMU 系）经 `imu^R_laser`、
  `imu^T_laser` 外参关联 LiDAR 系（`config/velodyne/vlp_16_calibration.yaml`，OpenCV FileStorage 格式）。
- 标定读取已支持 `extrinsicRotation_imu_camera` / `extrinsicTranslation_imu_camera`（`parameter.cpp:240-262`），
  当前无消费者，Phase 2 VIO 直接复用。
- 完整帧定义文档（`FRAMES_AND_CALIBRATION.md` + SE3 组合测试）按任务书 §5 在 Phase 2 动手前交付。

## 4. 关键参数（`config/vlp_16.yaml`）

- `use_imu_roll_pitch`、IMU 加速度限幅（`imu_acc_*_limit`）
- `feature_extraction_node`: scan_line=16, min_range=0.2, filter_point_size=3
- `laser_mapping_node`: 体素分辨率 0.1/0.2, max_iterations=5, localization_mode, 初始位姿
- `imu_preintegration_node`: `lidar_correction_noise=0.01`，IMU 噪声参数，`g_norm=9.80511`

## 5. 运行方式（本机）

```bash
~/d_livo/so_run3.sh    # 必须经此脚本（容器 + 6GB/6 核硬限制）；禁止裸跑 docker
```

产物：`super_odom_ws/results/{so_odom.txt, so_nodes.log, so_play.log, so_build.log}`。
数据：`datasets/superodom/vlp16_ros2`（仅 LiDAR+IMU）。

## 6. 已知行为记录（baseline 观察）

- 长时间运行正常，轨迹连续；bag 末段静止时反复出现
  `very small motion, not accumulating`（预期行为）。
- 运行期偶发 `failureDetected`（imu_preintegration 的图 reset 机制，上游固有），改造 Phase 5 时保持该机制可见。
- 本次增量构建 0 error（`results/build_check.log`）。
