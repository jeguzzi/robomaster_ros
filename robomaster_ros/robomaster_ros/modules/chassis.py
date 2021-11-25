import math
import quaternion

import rclpy.action

import sensor_msgs.msg
import geometry_msgs.msg
import robomaster_msgs.msg
import nav_msgs.msg

import robomaster.robot

from typing import Optional, List, Tuple, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import add_cb
from ..utils import deg, rad, rate

RADIUS = 0.05
RPM2SPEED = 2 * math.pi * 2 * RADIUS / 60
G = 9.81
MAX_ESC_ANGLE = 32767


# rmp -> [linear] speed
def wheel_speed(value: float) -> float:
    return RPM2SPEED * value


Esc = Tuple[List[int], List[int], List[int], List[int]]
SaStatus = Tuple[int, ...]


def esc2angle(value: int) -> float:
    return value / MAX_ESC_ANGLE * 2 * math.pi


def esc2angular_speed(value: int) -> float:
    return value / 60.0 * 2 * math.pi


def quaternion_from_euler(roll: float, pitch: float, yaw: float
                          ) -> quaternion.quaternion:
    # TODO(jerome):
    return quaternion.from_euler_angles(roll, pitch, yaw)


WHEEL_FRAMES = ['front_right_wheel_joint', 'front_left_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint']


class Chassis(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.clock = node.get_clock()
        self.logger = node.get_logger()
        self.node = node
        self.timeout: Optional[float] = node.declare_parameter("chassis.timeout", 0.0).value
        if self.timeout == 0.0:
            self.timeout = None
        self.api = robot.chassis
        odom_frame = node.tf_frame('odom')
        base_link = node.tf_frame('base_link')
        self.odom_msg = nav_msgs.msg.Odometry()
        self.odom_msg.header.frame_id = odom_frame
        self.odom_msg.child_frame_id = odom_frame
        self.imu_msg = sensor_msgs.msg.Imu()
        self.imu_msg.header.frame_id = base_link
        self.wheel_state_msg = sensor_msgs.msg.JointState(
            name=[node.tf_frame(name) for name in WHEEL_FRAMES])
        self.transform_msg = geometry_msgs.msg.TransformStamped(child_frame_id=base_link)
        self.transform_msg.header.frame_id = odom_frame
        self.odom_pub = node.create_publisher(nav_msgs.msg.Odometry, 'odom', 1)
        self.imu_pub = node.create_publisher(sensor_msgs.msg.Imu, 'imu', 1)
        self.chassis_state_pub = node.create_publisher(
            robomaster_msgs.msg.ChassisStatus, 'state', 1)
        chassis_rate = rate(node, 'chassis', 10)
        status_rate = rate(node, 'chassis.status', 1)
        if chassis_rate:
            self.api.sub_position(cs=1, freq=chassis_rate, callback=self.updated_position)
            self.api.sub_velocity(freq=chassis_rate, callback=self.updated_velocity)
            self.api.sub_attitude(freq=chassis_rate, callback=self.updated_attitude)
            self.api.sub_imu(freq=chassis_rate, callback=self.updated_imu)
            self.api.sub_esc(freq=chassis_rate, callback=self.updated_esc)
        if status_rate:
            self.api.sub_status(freq=status_rate, callback=self.updated_status)

        node.create_subscription(geometry_msgs.msg.Twist, 'cmd_vel', self.has_received_twist, 1)
        node.create_subscription(
            robomaster_msgs.msg.WheelSpeeds, 'cmd_wheels', self.has_received_wheel_speeds, 1)

        self._move_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.Move, 'move', self.execute_move_callback)

    def stop(self) -> None:
        self._move_action_server.destroy()
        if self.node.connected:
            self.api.drive_wheels(0, 0, 0, 0)
            self.api.unsub_position()
            self.api.unsub_velocity()
            self.api.unsub_attitude()
            self.api.unsub_imu()
            self.api.unsub_esc()
            self.api.unsub_status()

    def has_received_twist(self, msg: geometry_msgs.msg.Twist) -> None:
        self.api.drive_speed(
            x=msg.linear.x, y=-msg.linear.y, z=-deg(msg.angular.z), timeout=self.timeout)

    def has_received_wheel_speeds(self, msg: robomaster_msgs.msg.WheelSpeeds) -> None:
        self.api.drive_wheels(
            w1=wheel_speed(msg.front_right), w2=wheel_speed(msg.front_left),
            w3=wheel_speed(msg.rear_left), w4=wheel_speed(msg.rear_right),
            timeout=self.timeout)

    def updated_position(self, msg: Tuple[float, float, float]) -> None:
        position = self.odom_msg.pose.pose.position
        (position.x, position.y) = (msg[0], -msg[1])

    def updated_velocity(self, msg: Tuple[float, float, float, float, float, float]) -> None:
        velocity = self.odom_msg.twist.twist.linear
        (velocity.x, velocity.y) = (msg[0], -msg[1])

    # (yaw, pitch, roll)
    def updated_attitude(self, msg: Tuple[float, float, float]) -> None:
        orientation = self.odom_msg.pose.pose.orientation
        q = quaternion_from_euler(yaw=-rad(msg[0]), pitch=rad(msg[1]), roll=-rad(msg[2]))
        (orientation.x, orientation.y, orientation.z, orientation.w) = (q.x, q.y, q.z, q.w)

    # (acc, ang vel)
    def updated_imu(self, msg: Tuple[float, float, float, float, float, float]) -> None:
        # No angle, as of now
        # Check coppeliaSim IMU == REAL IMU == ROS conventions (0, 0, +G) when idle
        acceleration = self.imu_msg.linear_acceleration
        (acceleration.x, acceleration.y, acceleration.z) = [
            f * G * value for value, f in zip(msg[:3], (1, -1, -1))]
        angular_speed = self.imu_msg.angular_velocity
        (angular_speed.x, angular_speed.y, angular_speed.z) = [
            f * rad(value) for value, f in zip(msg[3:], (1, -1, -1))]
        # TODO(jerome): better? synchronization (should also check the jittering)
        stamp = self.clock.now().to_msg()
        self.imu_msg.orientation = self.odom_msg.pose.pose.orientation
        self.odom_msg.twist.twist.angular = self.imu_msg.angular_velocity
        self.odom_msg.header.stamp = stamp
        self.odom_pub.publish(self.odom_msg)
        self.imu_msg.header.stamp = stamp
        self.imu_pub.publish(self.imu_msg)
        position = self.odom_msg.pose.pose.position
        translation = self.transform_msg.transform.translation
        (translation.x, translation.y, translation.z) = (position.x, position.y, position.z)
        self.transform_msg.transform.rotation = self.odom_msg.pose.pose.orientation
        self.transform_msg.header.stamp = stamp
        self.node.tf_broadcaster.sendTransform(self.transform_msg)

    # (speeds + angles + timestamps + states)
    def updated_esc(self, msg: Esc) -> None:
        self.wheel_state_msg.position = [
            esc2angle(f * value) for value, f in zip(msg[1], (1, -1, -1, 1))]
        self.wheel_state_msg.velocity = [
            esc2angular_speed(f * value) for value, f in zip(msg[0], (1, -1, -1, 1))]
        self.wheel_state_msg.header.stamp = self.clock.now().to_msg()
        self.node.joint_state_pub.publish(self.wheel_state_msg)

    # (speeds + angles + timestamps + states)
    def updated_status(self, msg: SaStatus) -> None:
        keys = [key for key in robomaster_msgs.msg.ChassisStatus._fields_and_field_types.keys()
                if key != 'header']
        kwargs = {k: bool(value) for k, value in zip(keys, msg)}
        ros_msg = robomaster_msgs.msg.ChassisStatus(**kwargs)
        ros_msg.header.stamp = self.clock.now().to_msg()
        self.chassis_state_pub.publish(ros_msg)

    def execute_move_callback(self, goal_handle: Any) -> robomaster_msgs.action.Move.Result:
        # TODO(jerome): Complete with failures, ...  and velocity parameters
        request = goal_handle.request
        try:
            action = self.api.move(
                x=request.x, y=-request.y, z=deg(request.theta), xy_speed=request.linear_speed,
                z_speed=deg(request.angular_speed))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.Move.Result()

        self.logger.info(f'Start moving chassis with request {request}')
        feedback_msg = robomaster_msgs.action.Move.Feedback()

        def cb() -> None:
            feedback_msg.progress = action._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(action, cb)
        # while action.is_running:
        #     time.sleep(0.01)
        action.wait_for_completed()
        goal_handle.succeed()
        self.logger.info('Done moving chassis')
        return robomaster_msgs.action.Move.Result()
