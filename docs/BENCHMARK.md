# VIO Benchmark（TUM-VI room1，Phase 2c）

> 评测口径（总设计师 Gate 决定 A，固定不变）：
> ```text
> APE: SE(3)-aligned, no scale correction
> RPE: delta = 1 m
> ```
> Sim(3)（`--correct_scale`）只作尺度误差诊断，不作为成绩。
> GT：TUM-VI room1 官方 mocap ground truth（从
> `https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar`
> 的 `mav0/mocap0/data.csv` 提取，Euroc 格式 → TUM 格式转换，16541 帧，200Hz）。
> 评测段：初始化完成后至 bag 结束（截去 WARMUP 段）；时间戳匹配容差 0.02s。

## 双 profile 对照（Gate 决定 D：两份配置分开保留、分别跑）

| profile | 配置来源 | APE trans RMSE | APE max | RPE trans RMSE | RPE rot RMSE | solve p50/p95 |
|---|---|---:|---:|---:|---:|---|
| kalibr 官方链 | kalibr `T_cam_imu` 解析求逆 + TUM-VI 公开噪声 | **0.222 m** | 0.369 | 0.033 m | 0.72° | 14.5 / 20.7 ms |
| VINS upstream TUM | upstream `config/tum/tum_config.yaml` @ 90dabb5e 逐项（freq 10、min_dist 25、fisheye mask、作者外参、acc_n 0.04 等） | **0.227 m** | 0.431 | 0.029 m | 0.70° | 11.8 / 16.5 ms |

两条 profile 成绩在噪声水平内一致（Δ≈2%）→ **无参数调优空间被利用**，当前数字即诚实基线，
按 Gate 决定冻结。评测段均约 139s（kalibr）/137s（upstream），匹配 2739 / 1371 位姿。

## 尺度诊断（Sim(3)，仅诊断）

kalibr profile 加 `--correct_scale` 后 RMSE 0.222（与 SE(3) 结果无差别）→ 尺度因子 ≈1，
IMU 固定尺度工作正常，无尺度漂移问题。

## 运行环境

- 容器限制 `--memory=6g --cpus=6`（与 SuperOdom 主链基线相同硬件约束）
- 求解耗时：`processImage` 滑窗优化，滚动 500 帧 p50/p95，20Hz 帧预算 50ms，均实时余量充足

## 已知问题与处理记录

- upstream profile 首跑出现 NaN 级联崩溃：fisheye mask 使特征贴近等距模型有效域边缘，
  `liftProjective` 对域外点返回 NaN 进入求解器。处理：wrapper 边界过滤非有限观测
  （单一入口，vendor 代码零改动）；过滤后 upstream profile 全程 0 崩溃、0 ceres Terminating。

## 复现命令

```bash
# tracker+estimator（kalibr profile）
VIO_TRAJECTORY_CSV=/results/vio_tumvi_room1.csv \
  ros2 launch super_odometry_vio tumvi_vio.launch.py
# upstream profile
VIO_TRAJECTORY_CSV=/results/vio_tumvi_room1_upstream.csv \
  ros2 launch super_odometry_vio tumvi_vio_upstream.launch.py
# 评测（GT 转换见 datasets/tumvi/room1-groundtruth.csv）
evo_ape tum <est_active.tum> room1-groundtruth.tum -va --align --t_max_diff 0.02
evo_rpe tum <est_active.tum> room1-groundtruth.tum --align --delta 1 --delta_unit m \
  --t_max_diff 0.02 [-r angle_deg]
```

## 后续（按总设计师顺序，未执行）

1. room2/room3 held-out validation（换参不回退才算默认参数）
2. 小型 noise × keyframe_parallax 敏感性实验（最后才做）
