# VINS-Mono 移植记录（Phase 2c，Gate §3.2）

> upstream: https://github.com/HKUST-Aerial-Robotics/VINS-Mono.git @ `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`
> （与本地检出 `/home/peter/VINSmono_ws/src/VINS-Mono` 一致，该检出算法代码无本地改动，可干净对照）
> 许可：GPLv3，见 docs/THIRD_PARTY.md
> 目标：`algorithm diff ≈ 0`，仅 ROS/构建/参数兼容性差异。每个偏离 upstream 的改动都在下表登记。

## 1. 引入的文件清单

| upstream 文件 | 引入位置 | Modification | Reason |
|---|---|---|---|
| `vins_estimator/src/estimator.{h,cpp}` | `super_odometry_vio/src/estimator/` | **1 处 compat patch**（见 §3） | 构造器在栈对象上 UB（见 §3） |
| `vins_estimator/src/feature_manager.{h,cpp}` | 同上 | 无 | — |
| `vins_estimator/src/parameters.{h,cpp}` | 同上 | **adapted**：`readParameters(ros::NodeHandle&)` → `readParameters(const std::string& config_file)`；删除 readParam 模板 | ros::param → FileStorage 配置文件，字段读取逻辑逐行保持 |
| `vins_estimator/src/factor/{imu_factor.h, integration_base.h, projection_factor.{h,cpp}, projection_td_factor.{h,cpp}, marginalization_factor.{h,cpp}, pose_local_parameterization.{h,cpp}}` | `super_odometry_vio/src/estimator/factor/` | 无 | 纯 Eigen/Ceres |
| `vins_estimator/src/initial/{solve_5pts.{h,cpp}, initial_sfm.{h,cpp}, initial_alignment.h, initial_aligment.cpp, initial_ex_rotation.{h,cpp}}` | `super_odometry_vio/src/estimator/initial/` | 无 | 文件名 `initial_aligment.cpp`（拼写）系 upstream 原样 |
| `vins_estimator/src/utility/{utility.{h,cpp}, tic_toc.h}` | `super_odometry_vio/src/estimator/utility/` | 无 | — |
| （未引入）`utility/visualization.*`、`utility/CameraPoseVisualization.*`、`estimator_node.cpp` | — | 排除 | 纯 ROS1 可视化/节点编排，由本项目 ROS2 adapter 替代 |

## 2. ROS1 剥离方式（零算法文件改动）

`src/estimator/compat/` 提供 include 级 shim，vendored 文件的 `#include <ros/ros.h>`、
`<ros/console.h>`、`<ros/assert.h>`、`<std_msgs/Header.h>`、`<std_msgs/Float32.h>` 原样解析到 shim：

| shim | 内容 |
|---|---|
| `compat/ros/ros.h` | ROS_DEBUG/INFO/WARN/ERROR/FATAL(+STREAM) 宏 → stdout/stderr；ROS_ASSERT→assert、ROS_BREAK/COND 系列宏 |
| `compat/ros/console.h`、`compat/ros/assert.h` | 转发到 `ros/ros.h` shim |
| `compat/std_msgs/Header.h` | `std_msgs::Header` 存根：`stamp.toSec()`（vendored 代码只用这一项）|
| `compat/std_msgs/Float32.h` | 空存根（estimator.h include 但未使用）|

`parameters.cpp` 另加一行 `#include "ros/ros.h"`（原来经 parameters.h 间接获得日志宏）。

## 3. vendored 算法文件的全部改动（合计）

**estimator.cpp 构造器（唯一一处，初始化性 compat patch，不改任何算法）：**

```cpp
// 原： Estimator::Estimator(): f_manager{Rs} { ROS_INFO("init begins"); clearState(); }
Estimator::Estimator(): f_manager{Rs}
{
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
        pre_integrations[i] = nullptr;
    last_marginalization_info = nullptr;
    tmp_pre_integration = nullptr;
    ROS_INFO("init begins");
    clearState();
}
```

原因：`clearState()` 会 `delete` 这些裸指针。upstream 只以**全局对象**（BSS 零初始化）方式实例化
Estimator 才侥幸安全；栈/堆实例上首次 `clearState()` 就是 free 野指针（gdb 证实 SIGSEGV）。
补丁只做指针初始化，无任何算法语义变化。

## 4. Ceres 兼容性（Gate §4）

容器 Ceres 2.0.0（Ubuntu 22.04）：`ceres::LocalParameterization`（已弃用）仍可用。
**未做任何 API patch**，未改 residual/Jacobian/solver strategy。

## 5. 已发现的 upstream 特性（原样保留，测试锁定）

| 特性 | 证据 | 处理 |
|---|---|---|
| `Utility::deltaQ` 是一阶小角度四元数 `[1, θ/2]`（未归一化），非精确 SO(3) 指数 | `utility.h:16-28` | 原样保留；`test_vins_frames.DeltaQIsBodyFrameFirstOrderQuaternion` 锁定 |
| `PoseLocalParameterization` 所有 override 为 private | `pose_local_parameterization.h` | 经 ceres 基类接口调用（与 Ceres 一致）；测试亦如此 |
| `Estimator::is_valid` 成员从未被赋值（未初始化残留） | estimator.cpp 全文无赋值 | **禁止读取**；wrapper 状态判定只用 `solver_flag`+`failure_occur` |
| `FeaturePerFrame::parallax` 未在构造函数初始化 | `feature_manager.h` | wrapper 不读取该字段 |
| `processIMU(double dt, ...)` 接收**采样间隔**而非绝对时间；dt 由节点侧差分计算，`|dt|≥1s` 视为丢失跳过 | `estimator.cpp:91` + upstream node | wrapper `Impl::drainImu()` 复刻该语义 |
| 节点侧 IMU/特征按测量时间戳同步（getMeasurements）后才喂 estimator | upstream node | wrapper `processFeatures()` 同步排空 IMU ≤ 帧时刻，其余留缓冲；迟到帧丢弃计数 |

## 6. 帧约定（由 `test_vins_frames.cpp` 非 identity 用例锁定）

- `Rs[i]`、`Ps[i]` = `T_W_B`（body=IMU）；增量右乘（body 系）
- `ric`/`RIC[0]` = `R_B_C`，`tic`/`TIC[0]` = `p_B_C`
- 参数块布局 `[px,py,pz,qx,qy,qz,qw]`
- 投影链：`p_W = R_WB·(R_B_C·p_C + t_B_C) + p_WB`（`projection_factor.cpp` 原文语义）
- 与项目统一约定（docs/FRAMES_AND_CALIBRATION.md）一致，无需换算

## 7. 本项目新增（非 upstream，无对应文件）

| 文件 | 作用 |
|---|---|
| `include/super_odometry_vio/{vio_types.hpp, vio_estimator.hpp}` | Gate §7/8/15 接口类型与包装层 API |
| `src/wrapper/vio_estimator.cpp` | 状态机（WARMUP/ACTIVE/DEGRADED/BYPASS/RECOVERING）、H_visual 可观测性指标（J^T J 特征值）、relative pose（精确 stamp_i/stamp_j）、中央预测与外部深度钩子（仅存储，不消费）|
| `src/ros2/tracker_node.cpp`、`src/ros2/estimator_node.cpp` | ROS2 adapter（特征流格式与 upstream 节点逐字段一致）|
| `config/tum_vi_room1*.yaml`、`launch/tumvi_*.launch.py` | TUM-VI 配置（内参加源于 kalibr chain，见文件头注释）|

## 8. 信息流约束（Gate §5/§6 落实）

- VINS 内部 IMU 预积分/边缘化**全部保留**（未删未改）；
- 包装层对外只输出 `VioRelativePose`（相对位姿 + 质量），中央系统未来不得接触 VINS 内部因子；
- `setPosePrior`/`setStatePrediction`/`setExternalDepths` 钩子仅存储，Phase 2c 不消费（Gate §12/§15）。
