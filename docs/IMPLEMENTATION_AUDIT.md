# SuperOdom 本地仓库实现审计（Phase 1）

> 审计日期：2026-09-19
> 审计对象：`/home/peter/d_livo/super_odom_ws/src/SuperOdom`（下称"本仓库"）
> 依据：`AGENT_SUPER_ODOMETRY_2021.md`（IROS 2021 复现任务书）
> 结论先行：本仓库是 SuperOdom 的 **ROS2 slim 版**，已具备完整 LiDAR+IMU 闭环（LIO↔IMU 双向耦合 + LIO 退化处理 + IMU 中心最终输出），
> **完全没有视觉链路**。VIO 相关的接收端钩子（预测源切换、VIO 预测槽位）上游已预留，等待填充。

---

## 1. Git 状态（任务书 §2.1）

```text
路径     : /home/peter/d_livo/super_odom_ws/src/SuperOdom
HEAD     : f10e65cd50007767b22e4c401689665e20d827d6
分支     : ros2
remote   : https://github.com/superxslam/SuperOdom.git
status   :  M ros2_humble_docker/Dockerfile
           ?? livox_ros_driver2/          （内嵌独立 .git 的 vendored 驱动）
           ?? ros2_humble_docker/sodom_deps/（Livox-SDK2、Sophus 等离线依赖源码）
```

### 本地修改清单与处理策略

| 改动 | 内容 | 策略 |
|---|---|---|
| `ros2_humble_docker/Dockerfile` | pip/rosdep 换清华镜像源；`git clone` 依赖改为 `COPY sodom_deps/` 本地拷贝 | **保留**（环境适配，是容器能在此机器构建的原因） |
| `livox_ros_driver2/` | vendored Livox 驱动（自带 .git） | **保留，不纳入本次提交** |
| `ros2_humble_docker/sodom_deps/` | 离线依赖源码 | **保留，不纳入本次提交** |

未执行任何 `reset --hard` / 分支切换 / 工作区覆盖。

### 同机相关 checkout（与本任务的关系）

| 路径 | 分支 | 说明 | 策略 |
|---|---|---|---|
| `/home/peter/下载/jbgs/SuperOdom` | `macvio-stage8-working` | 用户实验线（macvo bridge / learned flow），大量本地改动 | **不触碰** |
| `/home/peter/VINSmono_ws/src/VINS-Mono` | HEAD `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` | VINS-Mono 上游（HKUST-Aerial-Robotics），仅 realsense 配置/launch 有本地改动，**算法代码干净** | Phase 2 VIO 复用来源，见 §6 |

## 2. 运行环境

- Docker 镜像 `superodom-ros2:latest`（ROS2 Humble），经 `/home/peter/d_livo/so_run3.sh` 运行，硬限制 `--memory=6g --cpus=6 --shm-size=256m`，FastDDS 禁 SHM、`ROS_LOCALHOST_ONLY=1`（此前不限内存曾把整机拖死，**不要裸跑**）。
- 构建验证（2026-09-19，容器内增量 colcon build）：`BUILD_EXIT:0`，6 packages finished，0 error。日志：`super_odom_ws/results/build_check.log`。
- 工作空间挂载：`src/SuperOdom → /root/ros2_ws/src`，build/install/log 均持久化在 `super_odom_ws/ws_*`。

## 3. 仓库结构审计（任务书 §2.2）

```text
SuperOdom/
├── super_odometry/                  # 主包（ROS2 Humble）
│   ├── include/super_odometry/
│   │   ├── FeatureExtraction/featureExtraction.h
│   │   ├── ImuPreintegration/imuPreintegration.h
│   │   ├── LaserMapping/laserMapping.h
│   │   ├── LidarProcess/LidarSlam.h
│   │   ├── LidarProcess/factor/{lidarOptimization.h, SE3AbsolutatePoseFactor.h,
│   │   │                            pose_local_parameterization.h}
│   │   ├── LidarProcess/LocalMap.h  +  LocalMap.h（include 根下另有一份）
│   │   ├── flann/{nanoflann.h, octree.h}
│   │   ├── config/parameter.h
│   │   └── utils/{EigenTypes.h, sophus_utils.hpp, superodom_utils.h, Twist.h, utility.h}
│   ├── src/
│   │   ├── FeatureExtraction/featureExtraction.cpp + featureExtraction_node.cpp
│   │   ├── ImuPreintegration/imuPreintegration.cpp + imuPreintegration_node.cpp
│   │   ├── LaserMapping/{laserMapping.cpp, lidarOptimization.cpp, LocalMap.cpp}
│   │   │   + laserMapping_node.cpp
│   │   ├── LidarProcess/{LidarSlam.cpp, pose_local_parameterization.cpp,
│   │   │                 SE3AbsolutatePoseFactor.cpp}
│   │   └── parameter/parameter.cpp
│   ├── config/{vlp_16.yaml, livox_mid360.yaml, os1_128.yaml, velodyne/, ouster/, livox/}
│   └── launch/{vlp_16.launch.py, os1_128.launch.py, livox_mid360.launch.py}
├── super_odometry_msgs/msg/{LaserFeature.msg, IterationStats.msg, OptimizationStats.msg}
├── livox_ros_driver2/               # vendored
├── ros2_humble_docker/Dockerfile
└── doc/                             # 上游 README 图片等
```

| 审计项 | 结论 |
|---|---|
| ROS2 package | ✅ 2 个自研包 + vendored livox 驱动 |
| LIO / LiDAR odometry node | ✅ `laserMapping_node`（特征提取→Ceres ICP→位姿输出） |
| IMU preintegration / IMU odometry node | ✅ `imu_preintegration_node`（GTSAM） |
| map representation | ✅ octree + LocalMap（`flann/octree.h`、`LidarProcess/LocalMap.h`），非论文所述 dynamic octree 但性能可用 |
| ICP / scan matching | ✅ Ceres 解析残差（`EdgeAnalyticCostFunction`/`SurfNormAnalyticCostFunction` + Tukey loss） |
| degeneracy detection | ✅ 见 §4 表第 4 行 |
| message definitions | ✅ LaserFeature / IterationStats / OptimizationStats（无任何视觉消息） |
| parameter files | ✅ yaml + `config/parameter.cpp` 读取（含 OpenCV `FileStorage` 读标定） |
| launch files | ✅ 3 个传感器各一 |
| docker / dependency setup | ✅ `ros2_humble_docker/Dockerfile`（用户已适配离线依赖） |
| tests | ❌ 无单元测试目录 |
| benchmark scripts | ❌ 无（结果靠 `so_run3.sh` 采样 odometry 到 txt） |

## 4. 论文能力 → 当前代码映射（任务书 §2.3）

| 2021 所需能力 | 当前代码位置 | 状态 | 决策 |
|---|---|---:|---|
| IMU preintegration | `src/ImuPreintegration/imuPreintegration.cpp`（`gtsam::PreintegratedImuMeasurements`，双实例 imuIntegratorOpt_/Imu） | 已有 | 复用 |
| IMU central odometry（中央图） | 同上：Prior(Pose/Vel/Bias)+ImuFactor+BetweenFactor(bias)+LIO 位姿校正；`failureDetected`→reset；发布最终 odometry | 已有 | 复用并扩展 |
| LiDAR odometry | `src/LaserMapping/laserMapping.cpp` + `src/LidarProcess/LidarSlam.cpp`（Ceres，DENSE_QR，Tukey） | 已有 | 保留不动 |
| LiDAR degeneracy handling | `LidarSlam.cpp:864` 起基于 `ceres::Covariance`(DENSE_SVD) 特征值分析；阈值 `Pos_degeneracy_threshold`/`Ori_degeneracy_threshold`（`laserMapping.cpp:109-110`）；退化时 `determinePredictionSource()` 切换预测源 | 已有 | 复用，接入新 ConstraintGate |
| IMU → LIO prediction | `laserMapping.cpp` `setInitialGuess()`：`initializeWithIMU()`（启动期）/ `selectPosePrediction()`（IMU_ORIENTATION、LIO_ODOM、VIO_ODOM、NEURAL_IMU_ODOM、CONSTANT_VELOCITY 五源切换）；`imu_odom_buf` 缓存 IMU odometry | 已有 | 复用 |
| LIO → IMU pose constraint | `imuPreintegration.cpp:449` `process_imu_odometry(lidarOdomTime, lidarPose)`，订阅 `/super_odometry/laser_odometry` 类话题做图校正 | 已有 | 复用 |
| Visual feature tracking | 无 | **缺失** | Phase 2 从 VINS-Mono 引入 |
| Sliding-window VIO | 无 | **缺失** | Phase 2 从 VINS-Mono 引入 |
| LiDAR depth → visual feature | 无 | **缺失** | Phase 3 新建 `lidar_depth_associator` |
| VIO reliability / covariance | 无 | **缺失** | Phase 2/6 新建 `vio_health` |
| VIO → IMU pose constraint | 无（中央图目前只有 LIO 校正） | **缺失** | Phase 5 新增 VIO factor |
| IMU → VIO prediction | 无 VIO 消费端；但 LIO 侧已预留 `pubVIOPrediction`（`ProjectName+"/vio_prediction"`） | **缺失** | Phase 4 |
| Final state publishing | `imuPreintegration.cpp` `pubImuOdometry`（`integrated_to_init`） | 已有（已是 IMU 中心） | 复用 |
| 系统健康诊断 | `pubHealthStatus`(Bool)、`OptimizationStats.msg` | 部分 | Phase 6 扩展为 health/detail 话题 |

### 上游已预留的 VIO 挂点（重要发现）

1. `laserMapping` 已发布 `~/vio_prediction`、`~/lio_prediction`、`~/prediction_source` 话题（`laserMapping.cpp:83-99`）。
2. `PredictionSource` 枚举含 `VIO_ODOM`、`NEURAL_IMU_ODOM`；`determinePredictionSource()` 中 **LIO 退化时优先消费 VIO 预测**（`laserMapping.cpp` `selectPosePrediction` 附近），当前 `sensorMeas.vio_prediction_status` 恒为 false（`laserMapping.cpp:684`），是待填充的槽位。
3. `sensor_data` 抽象（`sensorMeas`）已带 `vio/lio_prediction_status` 字段。
4. `parameter.cpp:240-262` 已会读 `extrinsicRotation_imu_camera` / `extrinsicRotation_imu_laser` 等标定（上游全量版遗留），VIO 可直接复用该标定读取。

## 5. 数据集现状（决定 Phase 2 验证路径）

| 数据集 | 传感器 | 可用性 |
|---|---|---|
| `datasets/superodom/vlp16_ros2/` | **仅** `/imu/data` + `/velodyne_points`，无相机，无 GT | SuperOdom 当前唯一 ROS2 数据，用于 Phase 0 baseline 与回归 |
| `datasets/lvi_sam/handheld.bag` (ROS1) | VLP-16 + 相机 + IMU | 有相机；需 ROS1→ROS2 转换后用于 VIO 验证 |
| `datasets/tumvi/dataset-room1_512_16.bag` (ROS1) | 相机 + IMU（无 LiDAR） | 标准 VIO smoke test（任务书 Phase 2 指定类型） |
| `datasets/ntu_viral/eee_03/eee_03.bag` (ROS1) | Ouster + 相机 + IMU | LiDAR+相机联合验证备选 |
| `datasets/hilti22/exp14_basement_2.bag` (ROS1) | Hesai + 5 相机 + IMU，有 GT | 退化场景 benchmark 候选 |

**Blocker 记录（任务书 §31 口径）**：当前 SuperOdom 配套数据无相机，VIO 无法在其上验证；
需转换一份含相机的 ROS1 bag（首选 TUM-VI 做独立 VIO smoke test，其次 LVI-SAM handheld 做 LIO+VIO 联合）。

## 6. 复用决策（任务书 §1.1 硬约束对照）

**禁止重写、直接复用**：GTSAM（中央图+预积分）、Ceres（LIO 优化）、PCL、Sophus、octree/nanoflann 地图、
现有 LIO 全部、现有 IMU preintegration/中央图、现有退化机制、rosbag/evo 等基础工具。

**Phase 2 VIO 复用来源**：本地 `/home/peter/VINSmono_ws/src/VINS-Mono`（upstream `HKUST-Aerial-Robotics/VINS-Mono`，
HEAD `90dabb5e`，GPLv3）。算法核心（feature_tracker / camera_models / feature_manager / estimator / initialization / marginalization）
代码干净可直接溯源；只引入任务书 §7.2 所列模块，不含 loop closure / relocalization。
按任务书 §7.3，以"VINS core library + ROS2 adapter"形态进入 `super_odometry/vio/`（或独立 core 目录，依构建体系定）。

## 7. 风险与待办

1. **相机数据 blocker**（§5）：Phase 2 前需完成 ROS1→ROS2 bag 转换（`rosbags` 工具），并确认相机内参
   （TUM-VI/LVI-SAM 均有公开标定）。
2. 中央图扩展（Phase 5）需在 `imuPreintegration` 的 reset/failure 逻辑中保证 VIO factor 与 LIO 校正
   不重复计数同一信息（任务书 §Phase 4）。
3. 现有 `imuPreintegration` 日志显示运行期偶发 `failureDetected`（见 baseline 记录），属上游已知 reset 行为，
   Phase 5 改造时不得掩盖该行为，需保留并记录诊断。
4. `include/super_odometry/LocalMap.h` 与 `include/super_odometry/LidarProcess/LocalMap.h` 疑似重复头文件，
   审计期间不改动，仅在触碰地图代码时核实。
