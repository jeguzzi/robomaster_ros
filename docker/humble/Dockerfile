FROM ros:humble
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    ros-humble-xacro \
    ros-humble-launch-xml \
    ros-humble-cv-bridge \
    ros-humble-launch-testing-ament-cmake \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-joy-linux \
    libopus-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN python3 -m pip install -U numpy numpy-quaternion pyyaml
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/jeguzzi/robomaster_ros.git src/robomaster_ros \
    && git clone https://github.com/ros/urdfdom.git --branch humble src/urdfdom \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/humble --merge-install \
    && rm -r /ros_ws
