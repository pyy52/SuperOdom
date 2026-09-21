# Local Reference for Codespace Agent

## Local OS

- **Host OS**: Ubuntu 20.04.6 LTS (x86_64, Linux kernel 5.15.0-139-generic)
- **Host CPU**: AMD Ryzen 7 7840H (16 vCPUs)
- **Host RAM**: 14 GB physical RAM (with 2 GB swap)
- **Important Note on OS**: The host machine runs Ubuntu 20.04 with ROS 1 Noetic. However, all ROS 2 development, compilation, and testing for SuperOdom were performed in a Docker container running **Ubuntu 22.04 (Jammy) + ROS 2 Humble**. The Codespace environment should therefore be set up targeting **Ubuntu 22.04 + ROS 2 Humble**.

## Previous Local ROS Setup

- **ROS distro**: ROS 2 Humble
- **Workspace path**: `/home/peter/d_livo/super_odom_ws` (source repository at `src/SuperOdom`)
- **Packages in workspace**:
  - `gtsam` (`src/SuperOdom/ros2_humble_docker/sodom_deps/gtsam`)
  - `livox_ros_driver2` (`src/SuperOdom/livox_ros_driver2`)
  - `livox_sdk2` (`src/SuperOdom/ros2_humble_docker/sodom_deps/Livox-SDK2`)
  - `sophus` (`src/SuperOdom/ros2_humble_docker/sodom_deps/Sophus`)
  - `super_odometry` (`src/SuperOdom/super_odometry`)
  - `super_odometry_msgs` (`src/SuperOdom/super_odometry_msgs`)
  - `super_odometry_vio` (`src/SuperOdom/super_odometry_vio`)
- **Build command used locally**:
  ```bash
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  colcon build --packages-select super_odometry super_odometry_vio --parallel-workers 1
  ```
  *(Crucial: `--parallel-workers 1` was required to prevent compiler OOM crashes).*
- **Launch command used locally**:
  ```bash
  ros2 launch super_odometry vlp_16.launch.py
  ```
- **Bag play command used locally**:
  ```bash
  ros2 bag play /home/peter/d_livo/datasets/superodom/vlp16_ros2 --rate 1.0
  ```
- **Important environment variables**:
  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
  ```
- **Special dependencies manually installed**:
  - GTSAM 4.2a0 / with custom preintegration
  - Sophus
  - Ceres Solver
  - PCL 1.12
  - OpenCV 4.5.4
  - Livox-SDK2
  - Boost (system, filesystem, date_time)
  - TBB

## Repository Context

- **GitHub repo**: `https://github.com/pyy52/SuperOdom.git`
- **Branch used locally**: `feat/phase4b2a-parity-reconcile` (also fast-forwarded to default branch `ros2`)
- **Is the local branch pushed to GitHub?**: Yes (`origin/ros2` and `origin/feat/phase4b2a-parity-reconcile` both match commit `ee1973d`).
- **Are there uncommitted local changes?**: No. All tracked files are clean (0 diff against `origin/ros2`).
- **Latest commit**: `ee1973d fix(parity-r2): output null for missing gate observations and make IMU gap visible in trace`

## Known Working Commands

```bash
# Source ROS2 Humble and Underlay
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Safe build with single worker (avoids OOM)
colcon build --packages-select super_odometry super_odometry_vio --parallel-workers 1

# Run unit tests
colcon test --packages-select super_odometry_vio
colcon test-result --test-result-base build/super_odometry_vio --verbose
# Expected: 87 tests, 0 errors, 0 failures

# Run parity sidecar tests
python3 -m unittest discover tools/parity/tests -v
# Expected: Ran 65 tests, OK

# Launch VLP-16 SuperOdom node
source install/setup.bash
ros2 launch super_odometry vlp_16.launch.py

# Play real VLP-16 bag
ros2 bag play /workspaces/data/bags/vlp16_ros2 --rate 1.0
```

## Known Problems

- **Compiler OOM**: Compiling Ceres/GTSAM templates across multiple parallel threads easily exhausts 8-16 GB RAM. Always use `--parallel-workers 1` or `--parallel-workers 2` unless Codespace has >=32 GB RAM.
- **Dynamic library path**: `libcephes-gtsam.so.1` or custom GTSAM builds require `export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH`.
- **IMU Preintegration Warmup**: On `vlp16_ros2`, the bag starts before the sensor moves. Initial frames may drop or trigger IMU initialization warnings until timestamps synchronize.

## Dataset Notes

- **Dataset root on local machine**: `/home/peter/d_livo/datasets/superodom/`
- **Recommended first dataset to upload**: `/home/peter/d_livo/datasets/superodom/vlp16_ros2`
- **Approximate size**: 1.3 GB
- **Bag format**: ROS 2 native sqlite3 storage (`metadata.yaml` + `vlp16_ros2.db3`)
- **Topics in bag**:
  - `/imu/data` (11,902 msgs, 200 Hz, `sensor_msgs/msg/Imu`)
  - `/velodyne_points` (1,555 msgs, 10 Hz, `sensor_msgs/msg/PointCloud2`)
- **Required config files**:
  - `super_odometry/config/vlp_16.yaml`
  - `super_odometry/config/velodyne/vlp_16_calibration.yaml`

## Proxy Notes

Local proxy is running and verified on:
- **HTTP/HTTPS Proxy**: `http://127.0.0.1:7897` (Clash Verge)
- Tested working via: `curl -I https://google.com --proxy http://127.0.0.1:7897`

Codespace cannot access local `127.0.0.1` directly.
If Codespace needs to access internet resources through the local proxy, set up a reverse SSH tunnel:

```bash
# On local machine (requires 'gh auth refresh -s codespace'):
gh codespace ssh -c "YOUR_CODESPACE_NAME" -- \
  -N \
  -o ExitOnForwardFailure=yes \
  -R 127.0.0.1:17890:127.0.0.1:7897
```

Inside Codespace:
```bash
export http_proxy="http://127.0.0.1:17890"
export https_proxy="http://127.0.0.1:17890"
export HTTP_PROXY="http://127.0.0.1:17890"
export HTTPS_PROXY="http://127.0.0.1:17890"
export all_proxy="http://127.0.0.1:17890"
export ALL_PROXY="http://127.0.0.1:17890"
export no_proxy="localhost,127.0.0.1,::1"
```

## Instruction to Codespace Agent

You are running inside GitHub Codespaces. Do not assume access to my local computer. Use this file only as reference. If you need local datasets, ask me to upload them to `/workspaces/data`. Do not commit datasets or secrets into git.

