# 坐标系与标定约定（任务书 §5）

> 本文是全项目唯一的帧约定权威来源。VIO/LIO/融合代码一律遵守；发现不一致先改文档共识再改代码，
> 并由 `super_odometry/test/test_frames.cpp` 单元测试锁定。

## 1. 帧定义

```text
W : world / odometry 帧（yaml: world_frame = "map"，另有旋转副本 map_rot）
B : body 帧 = IMU 帧
L : LiDAR 帧
C : camera 帧
```

## 2. 变换方向（唯一约定）

**T_X_Y 把 Y 系下表达的点/向量变换到 X 系：**

```text
p_W = T_W_B * p_B
p_B = T_B_L * p_L        # LiDAR → IMU
p_B = T_B_C * p_C        # camera → IMU
```

旋转块 R_X_Y 同方向：`v_X = R_X_Y * v_Y`。逆变换用 `.inverse()`，**禁止手写转置代替求逆**。

## 3. 本仓库现有标定文件的语义

`config/velodyne/vlp_16_calibration.yaml`（OpenCV FileStorage 格式）：

| 字段 | 值（vlp_16） | 语义（按文件内注释） | 对应本文记法 |
|---|---|---|---|
| `extrinsicRotation_imu_laser` | I | `imu^R_laser`：laser 系 → imu 系 | `R_B_L` |
| `extrinsicTranslation_imu_laser` | [0.08, 0.029, 0.03] | laser 原点在 imu 系下坐标 | `t_B_L` |
| `imu_laser_rotation_offset` | 0 | 附加旋转修正 | 见 `parameter.cpp` 消费逻辑 |
| `extrinsicRotation_imu_camera` / `..._imu_camera` | （暂无 vlp 数据） | camera 系 → imu 系 | `R_B_C` / `t_B_C` |

即标定读取侧（`src/parameter/parameter.cpp:240-262`，OpenCV `FileStorage` → `Eigen`，经 `Transformd`）与 §2 约定一致；
读取代码由现有 `parameter.cpp` 继续承担，VIO 不另建标定入口。

## 4. 代码内既有符号对照（现状备忘）

| 代码符号 | 位置 | 含义 |
|---|---|---|
| `imu_laser_R` / `T_i_c` | `parameter.cpp` / `parameter.h` | `R_B_L`；`T_i_c = T_B_C`（i=imu, c=camera） |
| `T_w_lidar`、`q_w_curr`、`t_w_curr` | `laserMapping.cpp` | `T_W_L`（LiDAR 在世界系位姿） |
| `sensorMeas.imuPrediction` | `laserMapping.cpp` | IMU 系姿态四元数（世界←IMU） |

> 注意：`laserMapping.cpp` 首帧初始化中 `q_w_curr = imu_laser_R.inverse() * q_imu`（`initializeFirstFrame`），
> 作用是把 IMU 世界姿态换算到 LiDAR 系；当前 vlp_16 标定 R=I 使两种读法不可区分。
> VIO 引入相机外参（一般非单位阵）后，任何歧义必须以 §2 约定 + 下方测试为准，禁止靠猜矩阵转置调通。

## 5. 中央状态

```text
X = { R_WB, p_WB, v_WB, b_a, b_g }        # 由中央 IMU odometry（imuPreintegration, GTSAM）维护并发布
```

第一版使用离线标定固定外参（本文件 §3），不做在线外参估计（任务书 §5、§34 非目标）。

## 6. 单元测试（锁定本文约定）

`super_odometry/test/test_frames.cpp`：

- `Se3Convention.CompositionAssociatesAcrossFrames`：`T_W_B*(T_B_L*T_L_C) == (T_W_B*T_B_L)*T_L_C` 及往返 `T⁻¹·T·p = p`
- `Se3Convention.DoubleInverseIsIdentity`
- `ExtrinsicConvention.Vlp16CalibMapsLaserOriginIntoImuTranslation`：vlp_16 标定值代入 `p_B = T_B_L·p_L`
- `ExtrinsicConvention.RotationMapsLaserVectorsIntoImuFrame`：方向性 + 逆方向
- `CameraProjection.PinholeProjectNormalizesByDepth`：针孔投影 `u = fx·X/Z + cx`（相机模型随 Phase 2 VINS core 引入后扩展）

运行：

```bash
# 容器内
colcon build --packages-select super_odometry
cd build/super_odometry && ctest -R test_frames --output-on-failure
```


## 7. TUM-VI room1 外参溯源（Gate 决定 D）

来源：官方 kalibr chain `openvins_ws/src/open_vins/config/tum_vi/kalibr_imucam_chain.yaml`
（cam0，`T_cam_imu` ≡ T_C_I，语义 `p_C = T_C_I · p_I`，即 R_C_I + p_C_I）。

VINS 需要的是 `p_I = T_I_C · p_C`：

```text
R_I_C = R_C_I^T
p_I_C = -R_C_I^T · p_C_I
```

原始矩阵（R_C_I）：
```text
[-0.9995250378696743  0.029615343885863205 -0.008522328211654736]
[ 0.0075019185074052044 -0.03439736061393144 -0.9993800792498829]
[-0.02989013031643309 -0.998969345370175  0.03415885127385616]
p_C_I = [0.04727988224914392, -0.047443232143367084, -0.0681999605066297]
```

求逆后（R_I_C，写入 `super_odometry_vio/config/tum_vi_room1_vio.yaml` 的 extrinsicRotation）：
```text
[-0.9995250378696743  0.0075019185074052044 -0.02989013031643309]
[ 0.029615343885863205 -0.03439736061393144 -0.998969345370175]
[-0.008522328211654736 -0.9993800792498829  0.03415885127385616]
p_I_C = [0.045574835649698026, -0.07116180183799704, -0.04468125411714437]
```

一致性检查（数值验证在 `test_vins_frames.TumViKalibrInverseConsistency`）：
`T_C_I · T_I_C ≈ I`（正交性残差 max 1.2e-15，det 1.0）。

注意区分（Gate 决定 D）：upstream VINS-Mono 自带 `config/tum/tum_config.yaml`（作者自标定外参+IMU 噪声）
作为独立 profile 保留：`config/tum_vi_room1_vio_upstream.yaml`，与本官方链 profile 分开跑、不合并。
