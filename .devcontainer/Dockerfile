# Devcontainer image for SuperOdom (2021-parity effort).
#
# Mirrors ros2_humble_docker/Dockerfile (the image the whole project is
# validated against locally) with two codespace-specific differences:
#
#  1. China-network adaptations are NOT carried over: pip uses default PyPI
#     and the three source dependencies are cloned from GitHub at the EXACT
#     commits the local image was built from (verified against the vendored
#     sodom_deps/ copies, which stay gitignored and untouched):
#       Livox-SDK2  @ 08f523c
#       Sophus      @ 97e71617
#       GTSAM 4.3.0 @ 4abef92   (GTSAM_USE_SYSTEM_EIGEN=ON, no MARCH_NATIVE)
#  2. Validation tooling used by this project is preinstalled: evo + rosbags
#     (benchmark/evaluation) and ros-humble-ament-cmake-gtest (unit tests).
#
# Local-only items intentionally omitted: torch/pandas/visdom (not needed by
# any package here), grid-map kept (upstream package.xml depends on it).
#
# NOTE: GUI (rviz2) is not usable in a headless codespace; validation is
# ctest / evo / CSV based, same as the resource-limited local runs.

FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake gdb git sudo vim less tmux curl wget unzip \
        libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev \
        libsuitesparse-dev libglew-dev \
        libceres-dev \
        python3-pip python3-tk \
        python3-rosdep python3-rosinstall python3-rosinstall-generator \
        python3-wstool \
        ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions \
        ros-humble-xacro ros-humble-robot-state-publisher \
        ros-humble-rviz2 ros-humble-image-transport \
        ros-humble-image-transport-plugins ros-humble-pcl-ros \
        ros-humble-grid-map-msgs ros-humble-grid-map \
        ros-humble-ament-cmake-gtest \
 && rm -rf /var/lib/apt/lists/*

# ---- Livox SDK2 (pinned) ---------------------------------------------------
WORKDIR /opt/deps
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
 && cd Livox-SDK2 && git checkout 08f523c \
 && mkdir build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install \
 && cd /opt/deps && rm -rf Livox-SDK2

# ---- Sophus (pinned, same commit as upstream SuperOdom Dockerfile) ---------
RUN git clone https://github.com/strasdat/Sophus.git \
 && cd Sophus && git checkout 97e71617 \
 && mkdir build && cd build \
 && cmake .. -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
 && make -j$(nproc) && make install \
 && cd /opt/deps && rm -rf Sophus

# ---- GTSAM 4.3.0 (pinned; system Eigen, no native dispatch: matches local) -
RUN git clone https://github.com/borglab/gtsam.git \
 && cd gtsam && git checkout 4abef92 \
 && mkdir -p build && cd build \
 && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
          -DGTSAM_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release .. \
 && make -j$(nproc) && make install \
 && cd /opt/deps && rm -rf gtsam
# GTSAM needs its lib path known at runtime.
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# ---- Python tooling for validation/benchmark -------------------------------
RUN pip3 install --no-cache-dir \
        numpy scipy matplotlib pyyaml \
        "setuptools<81" wheel \
        evo rosbags

# rosdep database (GitHub network is fast here, no mirror needed)
RUN rosdep update || true

# Code is expected at /workspaces/SuperOdom (GitHub codespace default).
WORKDIR /workspaces

# Default shell entry: ROS env sourced on every bash.
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc

CMD ["bash"]
