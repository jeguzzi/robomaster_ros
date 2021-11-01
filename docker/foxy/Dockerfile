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
    ros-foxy-joint-state-publisher \
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

# To speed up publishing python arrays
RUN git clone https://github.com/ros2/rosidl_python.git --branch galactic /ros_ws/src/rosidl_python
RUN git clone https://github.com/ros2/common_interfaces.git --branch foxy /ros_ws/src/common_interfaces
RUN git clone https://github.com/ros2/test_interface_files.git --branch foxy /ros_ws/src/test_interface_files

RUN cd /ros_ws && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select rosidl_generator_py sensor_msgs std_msgs geometry_msgs test_interface_files std_srvs nav_msgs && \
  rm -r build log

RUN cd /ros_ws/src && \
  git clone https://github.com/jeguzzi/robomaster_ros.git && \
  cd .. && \
  source /ros_entrypoint.sh && \
  colcon build --merge-install --packages-select robomaster_ros robomaster_msgs robomaster_description && \
  rm -r build log

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/ros_ws\/install\/local_setup.bash"' \
  /ros_entrypoint.sh
