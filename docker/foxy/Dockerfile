FROM ros:foxy
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    locales \
    python3-pip \
    ros-foxy-xacro \
    ros-foxy-joint-state-publisher \
    ros-foxy-joy \
    ros-foxy-joy-linux \
    ros-foxy-joy-teleop \
    libopus-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/berndpfrommer/ffmpeg_image_transport_msgs.git \
    && git clone https://github.com/ros/robot_state_publisher.git --branch foxy src/robot_state_publisher \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/opt\/ros\/ws\/setup.bash"' \
  /ros_entrypoint.sh

RUN python3 -m pip install -U numpy numpy-quaternion PyYaml av imageio
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=rm_libmedia_codec&subdirectory=lib/libmedia_codec"

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/jeguzzi/robomaster_ros.git src/robomaster_ros \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws