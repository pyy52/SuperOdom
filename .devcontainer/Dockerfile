FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake gdb git sudo vim less tmux curl wget unzip \
    libatlas-base-dev libeigen3-dev libpcl-dev libgoogle-glog-dev \
    libsuitesparse-dev libglew-dev libceres-dev \
    python3-pip python3-tk \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    ros-humble-tf2 ros-humble-cv-bridge ros-humble-pcl-conversions \
    ros-humble-xacro ros-humble-robot-state-publisher \
    ros-humble-rviz2 ros-humble-image-transport \
    ros-humble-image-transport-plugins ros-humble-pcl-ros \
    ros-humble-grid-map-msgs ros-humble-grid-map \
    ros-humble-ament-cmake-gtest \
 && rm -rf /var/lib/apt/lists/*

# Create non-root Codespaces user
RUN useradd -m -s /bin/bash vscode \
 && echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN mkdir -p /workspaces/SuperOdom && chown -R vscode:vscode /workspaces

WORKDIR /opt/deps

RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git \
 && cd Livox-SDK2 && git checkout 08f523c \
 && mkdir build && cd build \
 && cmake .. -DCMAKE_BUILD_TYPE=Release \
 && make -j$(nproc) && make install \
 && rm -rf /opt/deps/Livox-SDK2

RUN git clone https://github.com/strasdat/Sophus.git \
 && cd Sophus && git checkout 97e71617 \
 && mkdir build && cd build \
 && cmake .. -DBUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release \
 && make -j$(nproc) && make install \
 && rm -rf /opt/deps/Sophus

RUN git clone https://github.com/borglab/gtsam.git \
 && cd gtsam && git checkout 4abef92 \
 && mkdir build && cd build \
 && cmake -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
          -DGTSAM_BUILD_TESTS=OFF \
          -DCMAKE_BUILD_TYPE=Release .. \
 && make -j$(nproc) && make install \
 && rm -rf /opt/deps/gtsam

ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

RUN pip3 install --no-cache-dir \
    numpy scipy matplotlib pyyaml \
    "setuptools<81" wheel evo rosbags

RUN rosdep update || true

RUN echo 'source /opt/ros/humble/setup.bash' >> /home/vscode/.bashrc \
 && chown vscode:vscode /home/vscode/.bashrc

WORKDIR /workspaces

USER vscode

CMD ["bash"]
