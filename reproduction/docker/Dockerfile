ARG ROS_BASE=ros:noetic-ros-base-focal
FROM ${ROS_BASE}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git curl ca-certificates wget unzip \
    python3-catkin-tools python3-wstool python3-numpy \
    libboost-all-dev libeigen3-dev libpcl-dev libopencv-dev \
    libgoogle-glog-dev libgflags-dev libmetis-dev \
    ros-noetic-tf ros-noetic-cv-bridge ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions ros-noetic-xacro ros-noetic-robot-state-publisher \
    ros-noetic-rviz ros-noetic-rosbag ros-noetic-roslaunch ros-noetic-eigen-conversions \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch v2.3.0 https://github.com/Livox-SDK/Livox-SDK.git /tmp/livox-sdk \
    && git -C /tmp/livox-sdk checkout 7cf759a760ac674b7032c48b46ef8aaadab90383 \
    && cmake -S /tmp/livox-sdk -B /tmp/livox-sdk-build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build /tmp/livox-sdk-build --target livox_sdk_static -j2 \
    && cmake --install /tmp/livox-sdk-build \
    && rm -rf /tmp/livox-sdk /tmp/livox-sdk-build
WORKDIR /ws
