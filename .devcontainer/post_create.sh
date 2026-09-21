#!/usr/bin/env bash
set -e

echo "[setup] Starting Super Odometry Codespace setup"

# ---------------------------------------------------------------------------
# Persistent data directories (outside git)
# ---------------------------------------------------------------------------
mkdir -p /workspaces/data/bags
mkdir -p /workspaces/data/maps
mkdir -p /workspaces/data/datasets
mkdir -p /workspaces/data/results
ln -sfn /workspaces/data "$HOME/data"

# ---------------------------------------------------------------------------
# Source ROS + livox driver workspace in interactive shells
# ---------------------------------------------------------------------------
if [ -f /opt/ros/humble/setup.bash ]; then
  if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
  fi
fi
if [ -f /opt/livox_ws/install/setup.bash ]; then
  if ! grep -q "source /opt/livox_ws/install/setup.bash" "$HOME/.bashrc"; then
    echo "source /opt/livox_ws/install/setup.bash" >> "$HOME/.bashrc"
  fi
fi

sudo apt-get update

sudo apt-get install -y \
  git curl wget unzip tmux htop tree rsync ccache \
  build-essential cmake ninja-build gdb clang clangd lld \
  python3-pip python3-colcon-common-extensions \
  python3-vcstool python3-rosdep python3-argcomplete

if command -v rosdep >/dev/null 2>&1; then
  sudo rosdep init 2>/dev/null || true
  rosdep update || true
fi

# Install ROS package dependencies declared in package.xml.
# sophus and livox_ros_driver2 are not in the rosdistro index; they are built
# in the devcontainer image (see .devcontainer/Dockerfile), so skip them here.
if [ -f /opt/ros/humble/setup.bash ]; then
  . /opt/ros/humble/setup.bash
  rosdep install --from-paths super_odometry super_odometry_msgs super_odometry_vio \
    --ignore-src -r -y \
    --skip-keys "sophus livox_ros_driver2" \
    || echo "[setup] WARNING: rosdep reported unresolved keys (see log above)"
fi

pip3 install -U pip || true

echo "[setup] Finished Super Odometry Codespace setup"
