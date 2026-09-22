#!/bin/bash
# Codespace post-create: make a fresh clone buildable and green.
# Mirrors the local validated flow (colcon Release build + ctest).
set -euo pipefail

cd /workspaces/SuperOdom
source /opt/ros/humble/setup.bash

# livox_ros_driver2 is gitignored locally (vendored with its own .git), so a
# fresh codespace clone is missing it. Restore the exact upstream commit the
# local vendored copy points to.
if [ ! -d livox_ros_driver2 ]; then
  echo ">> cloning livox_ros_driver2 @ 4a1def9"
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git
  git -C livox_ros_driver2 checkout 4a1def9
  # Upstream ships ROS1/ROS2 variants selected via build.sh; our build needs
  # the ROS2 package.xml/CMakeLists (matches the locally vendored copy).
  cd livox_ros_driver2 && ./build.sh ROS2 && cd ..
fi

# Install any remaining declared dependencies (no-op when satisfied).
rosdep install --from-paths . --ignore-src -r -y || \
  echo ">> rosdep incomplete (expected for vendored/super packages); continuing"

# Colcon build in-repo (build/install/log are already .gitignore'd).
MAKEFLAGS=-j"$(nproc)" colcon build \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# Unit tests (frame conventions, camera model, tracker, lidar depth, shadow).
source install/setup.bash
for pkg in super_odometry super_odometry_vio; do
  echo ">> ctest ${pkg}"
  (cd build/${pkg} && ctest --output-on-failure) || {
    echo "!! tests failed for ${pkg}"; exit 1; }
done

echo ">> post-create done: workspace built and tests green"
