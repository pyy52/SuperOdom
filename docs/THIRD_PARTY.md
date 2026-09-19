# Third-party 代码溯源（任务书 §25）

> 开源准备项目：任何复用代码从引入第一天起在此登记。
> 约定：需要 VINS 的代码一律取自 VINS-Mono 原始 upstream（GPLv3，与本项目 GPL-3.0 路线一致）；
> LVI-SAM 只作算法思路参考，不直接复制其文件。

## 已复用（本仓库原有，随上游 SuperOdom 引入）

| 项目 | 用途 | 许可 | 备注 |
|---|---|---|---|
| GTSAM | IMU 预积分 + 中央因子图 | BSD | 系统依赖，未改源码 |
| Ceres Solver | LIO 优化 | BSD | 系统依赖，未改源码 |
| PCL / Eigen / Sophus | 点云/线性代数/流形 | BSD | 系统依赖 |
| nanoflann | 近邻搜索（`super_odometry/include/super_odometry/flann/`） | BSD | 随上游引入 |
| Livox ROS Driver 2 | 驱动（`livox_ros_driver2/`，vendored，自带 .git） | 上游许可 | 用户本地 vendored |

## 本次任务新增引入

| 项目 | upstream | upstream commit | 许可 | 引入文件 | 修改 | 版权声明 |
|---|---|---|---|---|---|---|
| VINS-Mono（核心算法） | https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git | `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`（与本地 `/home/peter/VINSmono_ws/src/VINS-Mono` 检出一致，本地无算法代码改动，可干净溯源） | GPLv3 | 见下表（按 Phase 2 进度逐项登记） | 仅做 ROS1→ROS2/接口适配，算法逻辑不动 | 保留上游 LICENSE 与版权头 |

### VINS-Mono 引入清单（随实现进度更新）

| upstream 文件 | 引入位置 | 状态 |
|---|---|---|
| `camera_model/include/camodocal/{camera_models,gpl,sparse_graph,calib,chessboard}/` | `super_odometry_vio/include/camodocal/` | 已引入（calib/chessboard 头随目录带入但未编译使用） |
| `camera_model/src/camera_models/*.cc`、`camera_model/src/gpl/*.cc` | `super_odometry_vio/src/` | 已引入，仅排除离线标定源（chessboard/、calib/），零算法修改 |
| `feature_tracker/src/feature_tracker.{h,cpp}` | `super_odometry_vio/src/tracker/` | 移植中（ROS1 依赖剥离） |
| `feature_tracker/src/parameters.{h,cpp}` | `super_odometry_vio/src/tracker/` | 移植中（ros::param → 配置结构体） |
| `vins_estimator/src/{estimator,feature_manager,factor,initial,utility}` | `super_odometry_vio/src/estimator/` | 待引入 |

## 参考但不复制

| 项目 | 参考内容 | 许可 | 说明 |
|---|---|---|---|
| LVI-SAM | LiDAR-assisted visual depth、VIS/LIS 耦合思路 | GPLv3 | 只读思路；其 visual 模块源自 VINS-Mono，需要代码时从 VINS-Mono upstream 取 |
| Super Odometry (IROS 2021) | 论文算法描述 | — | 本任务的目标规范 |
