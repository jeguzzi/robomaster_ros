FROM ros:lyrical
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    python3-pip \
    python3-numpy \
    python3-scipy \
    ros-lyrical-xacro \
    ros-lyrical-joint-state-publisher \
    ros-lyrical-joy \
    ros-lyrical-joy-teleop \
    ros-lyrical-joy-linux \
    ros-lyrical-urdfdom \
    ros-lyrical-ffmpeg-image-transport-msgs \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8

SHELL ["/bin/bash", "-c"]

RUN python3 -m pip install --break-system-packages numpy-quaternion PyYaml av
RUN python3 -m pip install --break-system-packages git+https://github.com/jeguzzi/RoboMaster-SDK.git
RUN python3 -m pip install --break-system-packages -i https://test.pypi.org/simple/ rm-libmedia-codec

RUN mkdir -p /ros_ws/src \
    && cd /ros_ws \
    && git clone https://github.com/jeguzzi/robomaster_ros.git src/robomaster_ros \
    && source /ros_entrypoint.sh \
    && colcon build --install-base /opt/ros/ws --merge-install \
    && rm -r /ros_ws

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/opt\/ros\/ws\/setup.bash"' \
  /ros_entrypoint.sh