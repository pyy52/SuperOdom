#!/bin/bash
set -e

# Phase 4B-1 R4 Reproducible Test Runner

# Get absolute path to the workspace directory
REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
WS_SRC_DIR=$(cd "$REPO_ROOT/.." && pwd)
WORKSPACE_DIR=$(cd "$WS_SRC_DIR/.." && pwd)

echo "Running Phase 4B-1 R4 Tests in Docker..."

docker run --rm \
    --volume="$WORKSPACE_DIR:/root/ros2_ws" \
    --workdir="/root/ros2_ws" \
    superodom-ros2:latest \
    /bin/bash -c "
        source /opt/ros/humble/setup.bash && \
        echo '--- Building SuperOdom VIO ---' && \
        colcon build --base-paths src/SuperOdom && \
        source install/setup.bash && \
        echo '--- Running Tests ---' && \
        colcon test --base-paths src/SuperOdom --packages-select super_odometry_vio && \
        echo '--- Test Results ---' && \
        colcon test-result --verbose
    "
