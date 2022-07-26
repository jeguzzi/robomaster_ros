import math
import quaternion
import time

import rclpy.action

import sensor_msgs.msg
import geometry_msgs.msg
import robomaster_msgs.msg
import nav_msgs.msg
import std_srvs.srv
import rcl_interfaces.msg

import robomaster.robot
import robomaster.action
import robomaster.protocol

from typing import Optional, List, Tuple, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import wait_action, abort_action
from ..utils import Rate, deg, rad, rate, nearest_rate

RADIUS = 0.05
AXIS = 0.2
RPM2SPEED = 2 * math.pi * RADIUS / 60
G = 9.81
MAX_ESC_ANGLE = 32767
DEFAULT_TWIST_ERROR_LINEAR = 0.005
DEFAULT_TWIST_ERROR_ANGULAR_XY = 0.01
DEFAULT_TWIST_ERROR_ANGULAR_Z = 0.03
DEFAULT_ACCELERATION_ERROR = 0.1


# rmp -> [linear] speed
def linear_speed_from_rpm(value: float) -> float:
    return RPM2SPEED * value


# [linear] speed -> rpm:
def rpm_from_linear_speed(value: float) -> int:
    return round(value / RPM2SPEED)


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


def wheel_speeds_from_twist(vx: float, vy: float, vtheta: float,
                            axis_length: float = AXIS) -> Tuple[float, float, float, float]:
    front_left  = vx - vy - axis_length * vtheta  # noqa
    front_right = vx + vy + axis_length * vtheta  # noqa
    rear_left   = vx + vy - axis_length * vtheta  # noqa
    rear_right  = vx - vy + axis_length * vtheta  # noqa
    return (front_left, front_right, rear_left, rear_right)


WHEEL_FRAMES = ['front_right_wheel_joint', 'front_left_wheel_joint',
                'rear_left_wheel_joint', 'rear_right_wheel_joint']


class Chassis(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.clock = node.get_clock()
        self.logger = node.get_logger()
        self.node = node
        self.robot = robot
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Set the command deadline in seconds. Values less or equal to zero "
                "disable the deadman check"))
        self.timeout: Optional[float] = node.declare_parameter(
            "chassis.timeout", 0.0, descriptor=desc).value
        if self.timeout == 0.0:
            self.timeout = None
        self.api = robot.chassis
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "When enabled, cmd_vel will control wheel state instead of chassis state."))
        self.twist_to_wheel_speeds: bool = node.declare_parameter(
            "chassis.twist_to_wheel_speeds", False, descriptor=desc).value
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "When enabled, forces the state estimation to be horizontal"))
        self.force_level: bool = node.declare_parameter(
            "chassis.force_level", False, descriptor=desc).value
        if self.twist_to_wheel_speeds:
            self.logger.info("topic cmd_vel will control wheel speeds")
        else:
            self.logger.info("topic cmd_vel will control chassis twist")
        odom_frame = node.tf_frame('odom')
        base_link = node.tf_frame('base_link')
        self.odom_msg = nav_msgs.msg.Odometry()
        self.odom_msg.header.frame_id = odom_frame
        self.odom_msg.child_frame_id = odom_frame
        self.imu_msg = sensor_msgs.msg.Imu()
        self.imu_msg.header.frame_id = base_link
        # no covariance for the pose, as there are no sensors measuring absolute
        # position or orientation. Twist covariance
        # we assume linear x, y have the same constant variance in world frame
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Linear speed variance in (m/s)^2"))
        self.linear_velocity_error = node.declare_parameter(
            "chassis.error.linear_velocity.xy", DEFAULT_TWIST_ERROR_LINEAR,
            descriptor=desc).value
        # we assume angular x, y have the same constant variance in world frame
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Non-horizontal angular speed variance in (m/s)^2"))
        self.angular_velocity_error_xy = node.declare_parameter(
            "chassis.error.angular_velocity.xy", DEFAULT_TWIST_ERROR_ANGULAR_XY,
            descriptor=desc).value
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Horizontal angular speed variance in (m/s)^2"))
        self.angular_velocity_error_z = node.declare_parameter(
            "chassis.error.angular_velocity.z", DEFAULT_TWIST_ERROR_ANGULAR_Z,
            descriptor=desc).value
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Whenever imu messages should include orientation"))
        self.imu_has_orientation = node.declare_parameter(
            "chassis.imu.include_orientation", True, descriptor=desc).value
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=(
                "Accelerometer variance in (m/s^2)^2"))
        self.linear_acceleration_error = node.declare_parameter(
            "chassis.error.linear_acceleration.xyz", DEFAULT_ACCELERATION_ERROR,
            descriptor=desc).value
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
            self.subscribe(chassis_rate)
        if status_rate:
            self.api.sub_status(freq=status_rate, callback=self.updated_status)

        node.create_subscription(geometry_msgs.msg.Twist, 'cmd_vel', self.has_received_twist, 1)
        node.create_subscription(
            robomaster_msgs.msg.WheelSpeeds, 'cmd_wheels', self.has_received_wheel_speeds, 1)

        # self.goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        self.action: Optional[robomaster.action.Action] = None
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._move_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.Move, 'move', self.execute_move_callback,
            cancel_callback=self.cancel_move_callback, callback_group=cbg)
        self.engage_server = node.create_service(std_srvs.srv.SetBool, 'engage_wheels',
                                                 self.engage_cb)
        node.add_on_set_parameters_callback(self.set_params_cb)

    @property
    def linear_velocity_error(self) -> float:
        return math.sqrt(self.odom_msg.twist.covariance[0])

    @linear_velocity_error.setter
    def linear_velocity_error(self, value: float) -> None:
        vs = self.odom_msg.twist.covariance
        vs[0] = vs[4] = value ** 2

    @property
    def angular_velocity_error_xy(self) -> float:
        return math.sqrt(self.odom_msg.twist.covariance[21])

    @angular_velocity_error_xy.setter
    def angular_velocity_error_xy(self, value: float) -> None:
        vs = self.odom_msg.twist.covariance
        vs[21] = vs[28] = value ** 2
        vs = self.imu_msg.angular_velocity_covariance
        vs[0] = vs[4] = value ** 2

    @property
    def angular_velocity_error_z(self) -> float:
        return math.sqrt(self.odom_msg.twist.covariance[35])

    @angular_velocity_error_z.setter
    def angular_velocity_error_z(self, value: float) -> None:
        vs = self.odom_msg.twist.covariance
        vs[35] = value ** 2
        vs = self.imu_msg.angular_velocity_covariance
        vs[8] = value ** 2

    @property
    def linear_acceleration_error(self) -> float:
        return math.sqrt(self.imu_msg.linear_acceleration_error[0])

    @linear_acceleration_error.setter
    def linear_acceleration_error(self, value: float) -> None:
        vs = self.imu_msg.linear_acceleration_covariance
        vs[0] = vs[4] = vs[8] = value ** 2

    def set_params_cb(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        for param in params:
            if 'chassis' not in param.name:
                continue
            if param.name == 'chassis.timeout':
                if param.value > 0:
                    self.timeout = param.value
                else:
                    self.timeout = None
            elif param.name == 'chassis.twist_to_wheel_speeds':
                self.twist_to_wheel_speeds = param.value
                if self.twist_to_wheel_speeds:
                    self.logger.info("topic cmd_vel will control wheel speeds")
                else:
                    self.logger.info("topic cmd_vel will control chassis twist")
            elif param.name == 'chassis.force_level':
                self.force_level = param.value
            elif param.name == 'chassis.rate':
                # TODO(Jerome): push the actual value back
                param.value = nearest_rate(param.value)
                self.subscribe(param.value)
            elif param.name == 'chassis.status':
                # TODO(Jerome): push the actual value back
                param.value = nearest_rate(param.value)
                self.api.sub_status(freq=param.value, callback=self.updated_status)
            elif param.name == 'chassis.imu_includes_orientation':
                self.imu_has_orientation = param.value
                if not self.imu_has_orientation:
                    self.imu_msg.orientation = [0.0, 0.0, 0.0]
            elif param.name == 'chassis.error.linear_velocity.xy':
                self.linear_velocity_error = param.value
            elif param.name == 'chassis.error.angular_velocity.xy':
                self.angular_velocity_error_xy = param.value
            elif param.name == 'chassis.error.angular_velocity.z':
                self.angular_velocity_error_z = param.value
            elif param.name == 'chassis.error.linear_acceleration.xyz':
                self.linear_acceleration_error = param.value
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def engage(self, value: bool) -> None:
        proto = robomaster.protocol.ProtoChassisSetWorkMode()
        proto._mode = 1 if value else 0
        self.api._send_sync_proto(proto)
        self.logger.info(f"{'Engaged' if value else 'Disengaged'} wheel motors")

    def engage_cb(self, request: std_srvs.srv.SetBool.Request,
                  response: std_srvs.srv.SetBool.Response) -> std_srvs.srv.SetBool.Response:
        self.engage(request.data)
        response.success = True
        return response

    def abort(self) -> None:
        if self.action:
            self.action._abort()
            while self.action is not None:
                self.logger.info("wait for the action to terminate")
                time.sleep(0.1)

    def subscribe(self, rate: Rate) -> None:
        if rate:
            # There is no need to unsubscribe
            self.api.sub_position(cs=1, freq=rate, callback=self.updated_position)
            self.api.sub_velocity(freq=rate, callback=self.updated_velocity)
            self.api.sub_attitude(freq=rate, callback=self.updated_attitude)
            self.api.sub_imu(freq=rate, callback=self.updated_imu)
            self.api.sub_esc(freq=rate, callback=self.updated_esc)

    def unsubscribe(self) -> None:
        self.api.unsub_position()
        self.api.unsub_velocity()
        self.api.unsub_attitude()
        self.api.unsub_imu()
        self.api.unsub_esc()

    def stop(self) -> None:
        self._move_action_server.destroy()
        self.engage_server.destroy()
        if self.node.connected:
            self.api.drive_wheels(0, 0, 0, 0)
            self.unsubscribe()
            self.api.unsub_status()

    def has_received_twist(self, msg: geometry_msgs.msg.Twist) -> None:
        if self.twist_to_wheel_speeds:
            front_left, front_right, rear_left, rear_right = wheel_speeds_from_twist(
                msg.linear.x, msg.linear.y, msg.angular.z)
            self.api.drive_wheels(
                w1=rpm_from_linear_speed(front_right), w2=rpm_from_linear_speed(front_left),
                w3=rpm_from_linear_speed(rear_left), w4=rpm_from_linear_speed(rear_right),
                timeout=self.timeout)
        else:
            self.api.drive_speed(
                x=msg.linear.x, y=-msg.linear.y, z=-deg(msg.angular.z), timeout=self.timeout)

    def has_received_wheel_speeds(self, msg: robomaster_msgs.msg.WheelSpeeds) -> None:
        self.api.drive_wheels(
            w1=rpm_from_linear_speed(msg.front_right), w2=rpm_from_linear_speed(msg.front_left),
            w3=rpm_from_linear_speed(msg.rear_left), w4=rpm_from_linear_speed(msg.rear_right),
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
        yaw = -rad(msg[0])
        if self.force_level:
            pitch = 0.0
            roll = 0.0
        else:
            pitch = rad(msg[1])
            roll = -rad(msg[2])
        q = quaternion_from_euler(yaw=yaw, pitch=pitch, roll=roll)
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
            f * value for value, f in zip(msg[3:], (1, -1, -1))]
        # TODO(jerome): better? synchronization (should also check the jittering)
        stamp = self.clock.now().to_msg()
        if self.imu_has_orientation:
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
        # TODO(jerome): Complete with ... velocity parameters
        # DONE(jerome): Complete with failures
        request = goal_handle.request
        try:
            self.action = self.api.move(
                x=request.x, y=-request.y, z=deg(request.theta), xy_speed=request.linear_speed,
                z_speed=deg(request.angular_speed))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.Move.Result()

        self.logger.info(f'Start moving chassis with request {request}')
        feedback_msg = robomaster_msgs.action.Move.Feedback()

        def cb() -> None:
            # if self.action._percent > 50:
            #     self.action._abort()
            #     return
            feedback_msg.progress = self.action._percent * 0.01  # type: ignore
            goal_handle.publish_feedback(feedback_msg)

        # add_cb(self.action, cb)
        # self.action.wait_for_completed()

        # while action.is_running:
        #     time.sleep(0.01)

        wait_action(self.action, cb)

        if goal_handle.is_cancel_requested:
            self.logger.info('Done moving chassis: canceled')
            goal_handle.canceled()
        elif self.action.has_succeeded:
            self.logger.info('Done moving chassis: succeed')
            goal_handle.succeed()
        else:
            self.logger.warning('Done moving chassis: aborted')
            try:
                goal_handle.abort()
            except rclpy._rclpy_pybind11.RCLError:
                # Happens when the context shutdown before we send feedbacks
                pass
        self.action = None
        return robomaster_msgs.action.Move.Result()

    def cancel_move_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        if self.action:
            self.node.executor.create_task(self._stop_action)
        return rclpy.action.CancelResponse.ACCEPT

    def _stop_action(self) -> None:
        if self.action:
            self.logger.info('Canceling move action')
            # ABORTED, which would be a more corrent state, is not handled as a completed state
            # by the SDK, therefore we use FAILED
            # Action specific way to abort:
            # self.action._changeto_state(robomaster.action.ACTION_FAILED)
            # self.api.move(x=0, y=0, z=0).wait_for_completed()
            # Action generic way to abort:
            abort_action(self.robot, self.action)

        # self.logger.warn('It is not possible to cancel onboard actions')
        # return rclpy.action.CancelResponse.REJECT
        # if self.action:
        #     self.logger.info('Canceling move action')
        #     self.action._abort()
        # return rclpy.action.CancelResponse.ACCEPT
