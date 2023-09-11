============
Installation
============

Pre-requisites
---------------

ROS2
^^^^

#. Install a current version of ROS2 (i.e., foxy, galactic, humble, or iron),
   following the `official instructions <https://docs.ros.org/en/humble/Installation.html>`_.

#. In case you opted for a minimal installation (ROS2-base), add the following packages:
   ``xacro, launch-xml, cv-bridge, launch-testing-ament-cmake, robot-state-publisher, joint-state-publisher-gui, joy, joy-teleop``

   .. code-block:: console

      sudo apt install \
      ros-<ROS_DISTRO>-xacro \
      ros-<ROS_DISTRO>-launch-xml \
      ros-<ROS_DISTRO>-cv-bridge \
      ros-<ROS_DISTRO>-launch-testing-ament-cmake \
      ros-<ROS_DISTRO>-robot-state-publisher \
      ros-<ROS_DISTRO>-joint-state-publisher \
      ros-<ROS_DISTRO>-joint-state-publisher-gui \
      ros-<ROS_DISTRO>-joy \
      ros-<ROS_DISTRO>-joy-teleop \
      ros-<ROS_DISTRO>-joy-linux

#. Install `colcon`_

   .. code-block:: console

      sudo apt install python3-colcon-common-extensions


Robomaster SDK
^^^^^^^^^^^^^^

Install `this fork <https://github.com/jeguzzi/RoboMaster-SDK>`_ of the official RoboMaster-SDK,
which fixes some issues of the upstream repo.

#. Install its dependencies

   .. code-block:: console

      sudo apt install libopus-dev python3-pip
      python3 -m pip install -U numpy numpy-quaternion pyyaml

#. Install the RoboMaster-SDK

   .. code-block:: console

      python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
      python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"


Robomaster ROS packages
-----------------------

#. In case you have none, `create a colcon workspace`_ where you want to build the packages,

   .. code-block:: console

      mkdir -p <ros2_ws>/src


#. Clone this repository

   .. code-block:: console

      cd <ros2_ws>/src
      git clone https://github.com/jeguzzi/robomaster_ros.git

#. Built the packages.

   .. code-block:: console

      cd <ros2_ws>
      colcon build



.. _colcon: https://colcon.readthedocs.io/en/released/
.. _create a colcon workspace: https://colcon.readthedocs.io/en/released/user/quick-start.html#tl-dr


Optional Runtime Dependencies
-----------------------------

To conform to ``image_transport`` when streaming the camera images, install ``PyAV`` and ``ffmpeg_image_transport``.

   .. code-block:: console

      python3 -m pip install av
      git clone https://github.com/berndpfrommer/ffmpeg_image_transport_msgs src/ffmpeg_image_transport_msgs
      git clone https://github.com/berndpfrommer/ffmpeg_image_transport src/ffmpeg_image_transport
      colcon build --packages-select ffmpeg_image_transport_msgs ffmpeg_image_transport


If you are running ``robomaster_ros`` on a different machine (e.g., a SCB connected to the robot), you can 
skip installing the ``image_transport`` plugin (i.e., ``ffmpeg_image_transport``) as you only need the interfaces (i.e., ``ffmpeg_image_transport_msgs``).
