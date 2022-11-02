# flake8: noqa: E402
import logging
import random
import time


import rclpy.logging
import rclpy.executors

# Disabled because it too expensive
# robomaster.logger = rclpy.logging.get_logger('sdk')

import robomaster
import robomaster.robot
import robomaster.protocol
import robomaster.conn
import robomaster.client


from robomaster_ros.modules import modules
from robomaster_ros.ftp import FtpConnection

import rclpy
import rclpy.node
import rclpy.action
import rclpy.task
import rclpy.duration

import std_msgs.msg
import sensor_msgs.msg

import tf2_ros.transform_broadcaster

from typing import Any, Optional


SERIAL_NUMBER_LENGTH = 14

def pad_serial(value: str) -> str:
  value = value[:SERIAL_NUMBER_LENGTH]
  value += '*' * (SERIAL_NUMBER_LENGTH - len(value))
  return value


def add_unknown_protocol(cmdset: int, cmdid: int, hint: str = '?') -> None:
    def unpack_resp(self: Any, buf: bytes, offset: int = 0) -> None:
        logging.debug(
            f'[{hint}] Received unknown response with cmd set {cmdset:#x} and id {cmdid:#x}, '
            f'with buffer {buf!r} ({len(buf)})')

    def unpack_req(self: Any, buf: bytes, offset: int = 0) -> None:
        logging.debug(
            f'[{hint}] Received unknown request with cmd set {cmdset:#x} and id {cmdid:#x}, '
            f'with buffer {buf!r} ({len(buf)})')

    _ = type(f'UnknownProtocol_{cmdset}_{cmdid}',
             (robomaster.protocol.ProtoData, ),
             {'_cmdset': cmdset,
              '_cmdid': cmdid,
              'unpack_resp': unpack_resp,
              'unpack_req': unpack_req})


def add_unknown_protocols() -> None:
    # state change
    add_unknown_protocol(0x3f, 0x29, 'CHASSIS STATE')
    # related to uart
    add_unknown_protocol(0x3f, 0xf4, 'SENSOR ADC')
    # related to tof
    add_unknown_protocol(0x24, 0x21, 'TOF')


add_unknown_protocols()

# add_unknown_protocol(0x3f, 0xb3)


def wait_for_robot(serial_number: Optional[str]) -> None:
    found = False
    while not found:
        try:
            found = robomaster.conn.scan_robot_ip(user_sn=serial_number)
        except OSError:
            pass
        if not found:
            time.sleep(random.uniform(1.0, 2.0))


class RoboMasterROS(rclpy.node.Node):  # type: ignore

    initialized: bool = False

    def __init__(self, executor: Optional[rclpy.executors.Executor] = None) -> None:
        super(RoboMasterROS, self).__init__("robomaster_ros", start_parameter_services=True)
        # robomaster.logger.set_level(logging.ERROR)
        lib_log_level : str = self.declare_parameter("lib_log_level", "ERROR").value.upper()
        robomaster.logger.setLevel(lib_log_level)
        conn_type: str = self.declare_parameter("conn_type", "sta").value[:]
        self.reconnect: bool = self.declare_parameter("reconnect", True).value
        sn: Optional[str] = self.declare_parameter("serial_number", "").value
        if sn:
            if len(sn) != SERIAL_NUMBER_LENGTH:
                sn = pad_serial(sn)
                self.get_logger().warn(
                    f"Serial number must have length {SERIAL_NUMBER_LENGTH}: "
                    f"trasformed to {sn}")
        else:
            sn = None
        self.connected = False
        if conn_type == 'sta':
            self.get_logger().info("Waiting for a robot")
            wait_for_robot(sn)
            self.get_logger().info("Found a robot")
        robomaster.conn.FtpConnection = FtpConnection
        # robomaster.conn.FtpConnection = FakeFtpConnection
        self.ep_robot = robomaster.robot.Robot()
        self.disconnection = rclpy.task.Future(executor=executor or rclpy.get_global_executor())
        # For now, to handle simulations without FTP
        self.get_logger().info(f"Try to connect via {conn_type} to robot with sn {sn}")
        try:
            self.ep_robot.initialize(conn_type=conn_type, sn=sn)
        except (AttributeError, TypeError):
            self.get_logger().error("Could not connect")
            self.disconnection.set_result(False)
            return
        self.get_logger().info("Connected")
        qos = rclpy.qos.QoSProfile(
            depth=1,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.connected_pub = self.create_publisher(std_msgs.msg.Bool, 'connected', qos)
        self.connected = True
        self._tf_name = self.declare_parameter('tf_prefix', '').value
        self.initialized = True
        self.heartbeat_check_timer = self.create_timer(5, self.heartbeat_check)
        self.heartbeat_handler = robomaster.client.MsgHandler(
            proto_data=robomaster.protocol.ProtoSdkHeartBeat(),
            ack_cb=lambda _, msg: self.got_heart_beat(msg))

        self.ep_robot._client.add_msg_handler(self.heartbeat_handler)
        self.joint_state_pub = self.create_publisher(
            sensor_msgs.msg.JointState, 'joint_states_p', 1)
        self.tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self.modules = [module(self.ep_robot, self) for name, module in
                        modules.items() if self.enabled(name)]
        module_string = ', '.join(type(module).__name__ for module in self.modules)
        self.get_logger().info(f"Enabled modules: {module_string}")
        self.connected_pub.publish(std_msgs.msg.Bool(data=True))

    def __del__(self) -> None:
        self.stop()

    def abort(self) -> None:
        if self.initialized:
            self.get_logger().info("Will abort any action")
            for module in self.modules:
                module.abort()

    def stop(self) -> None:
        if self.initialized:
            self.get_logger().info("Will stop client")
            self.heartbeat_check_timer.cancel()
            for module in self.modules:
                # self.get_logger().info(f"Will stop module {module}")
                module.stop()
                # self.get_logger().info(f"Stopped module {module}")
            time.sleep(0.5)
            if not self.connected:
                self.ep_robot._client.stop()
            self.ep_robot.close()
            self.connected = False
            self.connected_pub.publish(std_msgs.msg.Bool(data=False))
            self.initialized = False
            self.get_logger().info("Has stopped client")

    def tf_frame(self, name: str) -> str:
        if self._tf_name:
            return f'{self._tf_name}/{name}'
        return name

    def enabled(self, name: str) -> bool:
        return self.declare_parameter(f"{name}.enabled", False).value

    def heartbeat_check(self) -> None:
        self.connected = False
        self.disconnection.set_result(False)
        self.get_logger().warn("Disconnected")

    def got_heart_beat(self, msg: Any) -> None:
        self.heartbeat_check_timer.reset()
