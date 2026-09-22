#!/bin/bash
set -euo pipefail

cd /workspaces/SuperOdom
source /opt/ros/humble/setup.bash

if [ ! -d livox_ros_driver2 ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git
  git -C livox_ros_driver2 checkout 4a1def9
  cd livox_ros_driver2
  ./build.sh ROS2
  cd ..
fi

rosdep install --from-paths . --ignore-src -r -y || true

colcon build \
  --parallel-workers 4 \
  --cmake-args \
  -DCMAKE_BUILD_TYPE=Release

source install/setup.bash

for pkg in super_odometry super_odometry_vio; do
  if [ -d build/${pkg} ]; then
    (cd build/${pkg} && ctest --output-on-failure)
  fi
done

echo "SuperOdom ROS2 Codespace ready."
