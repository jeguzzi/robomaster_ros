FROM ros:foxy
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    ros-foxy-xacro \
    ros-foxy-launch-xml \
    ros-foxy-cv-bridge \
    ros-foxy-launch-testing-ament-cmake \
    ros-foxy-robot-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-joy \
    ros-foxy-joy-teleop \
    libopus-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN python3 -m pip install -U numpy numpy-quaternion pyyaml
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"

COPY ./robomaster/robomaster_msgs /ros_ws/src/robomaster_msgs

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select robomaster_msgs

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/local_setup.bash"' \
  /ros_entrypoint.sh

COPY ./robomaster/robomaster_description /ros_ws/src/robomaster_description

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select robomaster_description

COPY ./robomaster/robomaster_ros /ros_ws/src/robomaster_ros

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select robomaster_ros
