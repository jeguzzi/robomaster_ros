Robomaster-ROS
==============

This repository contains a ROS2 driver for the DJI Robomaster family of robots (EP and S1) based on the [official Python client library](https://github.com/dji-sdk/RoboMaster-SDK).

Full documentation available at https://jeguzzi.github.io/robomaster_ros.


## Installation

### Pre-requisites

#### ROS2

Install a currently supported version of ROS2 (foxy -- iron), following the [official instructions](https://docs.ros.org/en/galactic/Installation.html).
and then install colcon
```bash
sudo apt install python3-colcon-common-extensions
```

If you just install ROS2-base, add also the following packages:
```
xacro, launch-xml, cv-bridge, launch-testing-ament-cmake, robot-state-publisher, joint-state-publisher-gui, joy, joy-teleop
```
```bash
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
```

#### Robomaster SDK

Install [this fork](https://github.com/jeguzzi/RoboMaster-SDK) of the official RoboMaster-SDK, which fixes some issues of  the upstream repo.

First install its dependencies `libopus-dev`
```bash
sudo apt install libopus-dev python3-pip
```
and
```bash
python3 -m pip install -U numpy numpy-quaternion pyyaml
```
then install the RoboMaster-SDK
```bash
python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git
python3 -m pip install git+https://github.com/jeguzzi/RoboMaster-SDK.git#"egg=libmedia_codec&subdirectory=lib/libmedia_codec"
```

### ROS2 package

Create a `colcon` package where you want to build the packages, clone this repository, and built the packages.
```
mkdir -p <ros2_ws>/src
git clone https://github.com/jeguzzi/robomaster_ros.git
cd <ros2_ws>
source /opt/ros/<ROS_DISTRO>/setup.bash
colcon build
```

## Usage

Use one of the two launch files `{s1|ep}.launch` to launch the driver and the robot model.
```bash
cd <ros2_ws>
source install/setup.bash
ros2 launch robomaster_ros {s1|ep}.launch
```

We also provide docker images. Check [the documentation](docker.md) for their usage.

### Arguments

The launch files accept a list of arguments
```bash
ros2 launch robomaster_ros {s1|ep}.launch <key_1>:=<value_1> <key_2>:=<value_2> ...
```


#### Common Configurations

The two different robot models share some configuration.

| key              | type    | valid values              | default | description                                                                                           |
| ---------------- | ------- | ------------------------- | ------- | ----------------------------------------------------------------------------------------------------- |
| name             | string  | valid ROS names           | ''      | a name used as ROS namespace                                                                          |
| serial_number    | string  | 8 character ascii strings | ''      | the serial number of the robot, leave empty to connect to the first robot found                             |
| conn_type        | string  | ap, rndis, sta            | sta     | the connection network type: managed/router (sta); robot's access point (ap); usb (rndis)             |
| lib_log_level    | string  | DEBUG, INFO, WARN, ERROR  | ERROR   | the log-level used by the internal Robomaster API                                                     |
| video_resolution | integer | 360, 540, 720             | 360     | the video [vertical] resolution: 640x360 (360);  960x540 (540);    1280x720 (720)                     |
| video_raw        | bool    |                           | true    | whether to publish the raw [decompressed] images to the topic `<name>/camera/image_raw`               |
| video_h264       | bool    |                           | false   | whether to publish the original h264 video stream to the topic `<name>/camera/image_h264`             |
| video_compressed | bool    |                           | false   | whether to publish the compressed [jpeg] images to the topic `<name>/camera/image_raw/compressed`         |
| audio_raw        | bool    |                           | true    | whether to publish the raw [decompressed] audio to the topic `<name>/camera/audio_raw`                    |
| audio_opus       | bool    | | true                      |          whether to publish the original [compressed] opus audio stream to the topic `<name>/camera/audio_opus` |
| chassis_rate     | int     | 1, 5, 10, 20, 50          | 10      | the rate [Hz] at which to publish the odometry                                                                 |
| joint_state_rate | int     | 1, 5, 10, 20, 50          | 10      | the rate [Hz] at which to publish aggregated joint states                                                      |
| sensor_adapter   | bool    |                           | false   | Whether at least one sensor adapter (IO) is connected and should be published to `<name>/...`         |
| sensor_adapter_rate                 | int        |  1, 5, 10, 20, 50                         |  10       | the rate [Hz] at which to publish the sensor adapter values                                       

#### S1-specific Configurations

Some configurations are specific for the Robomaster S1.

| key         | type    | valid values     | default | description                                       |
| ----------- | ------- | ---------------- | ------- | ------------------------------------------------- |
| gimbal_rate | integer | 1, 5, 10, 20, 50 | 10      | the rate [Hz] at which to gather the gimbal state |
|   display_battery          |  string     |  off, right, left |   off      |   whether and where to display the battery state: do not display (on); display on the right gimbal led (right); display on the left gimbal led (left)                                             |

#### EP-specific Configurations

Some configurations are specific for the Robomaster EP.

| key                   | type | valid values | default | description                                                                                                                |
| --------------------- | ---- | ------------ | ------- | -------------------------------------------------------------------------------------------------------------------------- |
| left_motor_zero       | int  |              | 1242    | the [arm] left servo motor encoder value at zero angle                                                                           |
| right_motor_zero      | int  |              | 1273    | the [arm] right servo motor encoder value at zero angle                                                                          |
| left_motor_direction  | int  | -1, 1        | -1      | the [arm] left servo motor direction: angle increases when encoder increases (+1);  angle decreases when encoder increases (-1)  |
| right_motor_direction | int  | -1, 1        | -1      | the [arm] right servo motor direction: angle increases when encoder increases (+1);  angle decreases when encoder increases (-1) |


### Multiple robots

If you want to control multiple robots through ROS, you need to know their serial numbers and set a different name for each of them (names are used as ROS namespaces and as `tf` prefixes). For physical robots, the serial number is written on top of the intelligent controller. For simulated robots, you set the serial number when you launch the simulation.

For example, we assume that you are using two [simulated] S1 robots with serial numbers `"RM0"` and `"RM1"`, and that you want to use the serial numbers also as names. In two consoles, launch
```bash
cd <ros2_ws>
source install/setup.bash
ros2 launch robomaster_ros s1.launch name:=RM0 serial_number:=RM0
```
and
```bash
cd <ros2_ws>
source install/setup.bash
ros2 launch robomaster_ros s1.launch name:=RM1 serial_number:=RM1
```
Then, for instance, you can make the robots spin in opposite direction by publishing to their respective topics:
```
ros2 topic pub /RM0/cmd_vel geometry_msgs/msg/Twist "{angular: {z: -0.5}}" --once
ros2 topic pub /RM1/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once
```


### Thanks

This work has been supported by the European Commission through the Horizon 2020 project [1-SWARM](https://www.1-swarm.eu/), grant ID 871743.
