# How to deploy the ROS2 Robomaster driver via docker

For all of you that would like to use one of our Robomasters, here you find the steps for installing and running the software on your machine, e.g., to collect data outside of the lab.

## Installation

1. Install docker and docker-compose https://docs.docker.com/get-docker ; community edition (CE) is available for free for mac, win, and linux.

2. Download the docker-compose.yaml for the relevant model (EP or S1) from `docker/<ROS_VERSION>/{ep|s1}/docker-compose.yaml` .

3. Pull the pre-built docker image
```bash
    cd <folder containing the docker-compose.yaml file>
    docker-compose pull
```

### Note on EOL ROS2 versions

Building docker image linked to an EOL ROS2 version (like `foxy` and `galactic`) could lead to errors because the related repository signature is no more valid. We leave the docker files for reference but we strongly suggest to only build images linked to [currently supported ROS2 versions](https://docs.ros.org/en/foxy/Releases.html).


## Configuration

To configure the driver, add/remove/edit the key-value pairs at the end of line 6 of the compose file
```yaml
    command: ros2 launch robomaster_ros {s1|ep}.launch <key_1>:=<value_1> <key_2>:=<value> ...
```

See the [launch file documentation](README.md) for further info.


## Running

The compose file launches two containers, one with the driver and one to teleoperate the robot with a joypad (connect the joypad first!).

To bring up both driver and teleop:
```bash
cd <folder containing the docker-compose.yaml file>
docker-compose up
```

To bring up just the driver or the teleop
```bash
cd <folder containing the docker-compose.yaml file>
docker-compose up {driver|teleop}
```
