#
# Super Odometry — self-contained ROS2 Humble devcontainer image.
#
# NOTE: this intentionally does NOT reuse ros2_humble_docker/Dockerfile, because
# that Dockerfile COPYs third-party sources from ros2_humble_docker/sodom_deps/
# which is gitignored (not present in a fresh clone). Instead all third-party
# dependencies are fetched here from upstream repos at pinned commits/tags so
# the image is reproducible from the repo alone.
#
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# ---------------------------------------------------------------------------
# System + ROS build dependencies (binaries from apt where possible)
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake ninja-build git wget unzip curl ca-certificates \
        sudo vim tmux htop tree rsync less gdb clang clangd ccache lld \
        libeigen3-dev libpcl-dev libceres-dev libsuitesparse-dev libtbb-dev \
        libgoogle-glog-dev libatlas-base-dev libglew-dev libboost-system-dev \
        python3-pip python3-colcon-common-extensions python3-vcstool \
        python3-rosdep python3-rosinstall python3-argcomplete \
        ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-geometry-msgs \
        ros-humble-cv-bridge ros-humble-pcl-conversions ros-humble-pcl-ros \
        ros-humble-xacro ros-humble-robot-state-publisher \
        ros-humble-image-transport ros-humble-image-transport-plugins \
        ros-humble-rviz2 ros-humble-rviz-2d-overlay-plugins \
        ros-humble-grid-map-msgs ros-humble-grid-map \
        ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

# Python deps used by helper/visualization scripts
RUN pip3 install --no-cache-dir numpy scipy pyyaml

# ---------------------------------------------------------------------------
# Livox SDK2 (pinned)
# ---------------------------------------------------------------------------
ARG LIVOX_SDK2_SHA=08f523c930b2f0ba1e98a6afaa8d7476bf479908
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 \
    && cd /tmp/Livox-SDK2 && git checkout ${LIVOX_SDK2_SHA} \
    && mkdir build && cd build \
    && cmake .. && make -j2 && make install \
    && rm -rf /tmp/Livox-SDK2 && ldconfig

# ---------------------------------------------------------------------------
# Sophus (pinned, matches the vendored version used by the upstream Dockerfile)
# ---------------------------------------------------------------------------
ARG SOPHUS_SHA=d0b7315a0d90fc6143defa54596a3a95d9fa10ec
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus \
    && cd /tmp/Sophus && git checkout ${SOPHUS_SHA} \
    && mkdir build && cd build \
    && cmake .. -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
    && make -j2 && make install \
    && rm -rf /tmp/Sophus && ldconfig

# ---------------------------------------------------------------------------
# GTSAM 4.2.0 (system Eigen, no native tuning flags for portability)
# ---------------------------------------------------------------------------
RUN git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git /tmp/gtsam \
    && cd /tmp/gtsam && mkdir -p build && cd build \
    && cmake .. \
        -DGTSAM_USE_SYSTEM_EIGEN=ON \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
        -DGTSAM_BUILD_UNSTABLE=ON \
    && make -j2 && make install \
    && rm -rf /tmp/gtsam && ldconfig

# ---------------------------------------------------------------------------
# livox_ros_driver2 (not released to rosdistro; build in a dedicated workspace)
# ---------------------------------------------------------------------------
ARG LIVOX_DRIVER2_SHA=4a1def929e5b59c7a8122d19fce6efba581ce9f7
RUN mkdir -p /opt/livox_ws/src \
    && git clone https://github.com/Livox-SDK/livox_ros_driver2.git /opt/livox_ws/src/livox_ros_driver2 \
    && cd /opt/livox_ws/src/livox_ros_driver2 && git checkout ${LIVOX_DRIVER2_SHA} \
    && rm -f package.xml && cp package_ROS2.xml package.xml \
    && rm -rf launch && cp -r launch_ROS2 launch
RUN cd /opt/livox_ws \
    && . /opt/ros/humble/setup.bash \
    && colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble

# Make ROS + livox driver workspace available in interactive shells
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "source /opt/livox_ws/install/setup.bash" >> /root/.bashrc
