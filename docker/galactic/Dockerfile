FROM ros:galactic
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    ros-galactic-xacro \
    ros-galactic-launch-xml \
    ros-galactic-cv-bridge \
    ros-galactic-launch-testing-ament-cmake \
    ros-galactic-robot-state-publisher \
    ros-galactic-joint-state-publisher \
    # ros-galactic-joint-state-publisher-gui \
    ros-galactic-joy \
    ros-galactic-joy-teleop \
    ros-galactic-joy-linux \
    libopus-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN python3 -m pip install -U numpy numpy-quaternion pyyaml
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"

# To speed up publishing python arrays

RUN mkdir -p /ros_ws/src && cd /ros_ws/src && \
  git clone https://github.com/ros2/rosidl_python.git --branch galactic && \
  git clone https://github.com/ros2/common_interfaces.git --branch galactic && \
  git clone https://github.com/ros2/test_interface_files.git --branch galactic && \
  cd .. && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select rosidl_generator_py sensor_msgs std_msgs geometry_msgs test_interface_files std_srvs nav_msgs && \
  rm -r build log src

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/local_setup.bash"' \
  /ros_entrypoint.sh

RUN mkdir -p /ros_ws/src && cd /ros_ws/src && \
  git clone https://github.com/jeguzzi/robomaster_ros.git && \
  cd .. && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select robomaster_ros robomaster_msgs robomaster_description && \
  rm -r build log src
