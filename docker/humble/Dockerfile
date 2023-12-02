FROM ros:humble
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    python3-pip \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-joint-state-publisher \
    ros-humble-joy \
    ros-humble-joy-teleop \
    ros-humble-joy-linux \
    libopus-dev \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/ros/urdfdom.git --branch humble src/urdfdom \
    && git clone https://github.com/berndpfrommer/ffmpeg_image_transport_msgs.git \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/opt\/ros\/ws\/setup.bash"' \
  /ros_entrypoint.sh

RUN python3 -m pip install -U numpy numpy-quaternion PyYaml av
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/jeguzzi/robomaster_ros.git src/robomaster_ros \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws

