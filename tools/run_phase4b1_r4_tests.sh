#!/usr/bin/env bash
set -euo pipefail

# Find the repository root (SuperOdom)
REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
WORKSPACE_DIR=$(cd "$REPO_ROOT/../.." && pwd)

echo "Running Phase 4B-1 R4 Tests in Docker..."

docker run --rm \
    --volume="$WORKSPACE_DIR:/root/ros2_ws" \
    --workdir="/root/ros2_ws" \
    superodom-ros2:latest \
    /bin/bash -c "
        git config --global --add safe.directory /root/ros2_ws && \
        git config --global --add safe.directory /root/ros2_ws/src/SuperOdom && \
        source /opt/ros/humble/setup.bash && \
        echo '--- Building SuperOdom VIO ---' && \
        colcon build --base-paths src/SuperOdom && \
        export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH} && \
        source install/setup.bash && \
        echo '--- Running Tests ---' && \
        colcon test --base-paths src/SuperOdom && \
        echo '--- Test Results ---' && \
        colcon test-result --all --verbose
    "
