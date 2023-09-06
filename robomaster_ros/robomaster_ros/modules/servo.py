import math
import yaml
import time

import rclpy.action

import robomaster.robot
import sensor_msgs.msg
import robomaster_msgs.msg
import robomaster_msgs.srv
import robomaster_msgs.action

from typing import Dict, List, Tuple, Any, Optional, cast
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate, deg, rad
from ..action import add_cb

# DONE from the APP 1 unit = 0.17578 degress
# RAD2SERVO = 325.95
# 1024 <-> 180 deg
UNIT_180_DEG = 1024
MAX_RPM = 49

ServoData = Tuple[List[int], List[int], List[int]]


def rpm(angular_speed: float) -> float:
    return max(min(MAX_RPM, 30 * angular_speed / math.pi), -MAX_RPM)


class RMServo:
    index: int
    valid: bool
    value: int
    speed_value: int
    direction: int  # Literal[-1, 1]
    name: str

    def __init__(self, index: int, reference_angle: float, reference_value: int, direction: int,
                 name: str = '') -> None:
        self.index = index
        self.valid = False
        if direction > 0:
            self.direction = 1
        else:
            self.direction = -1
        self.reference_angle = reference_angle
        self.reference_value = reference_value
        self.name = name

    @property
    def angle(self) -> float:
        return (self.reference_angle +
                self.direction * (self.value - self.reference_value) * math.pi / UNIT_180_DEG)

    @property
    def speed(self) -> float:
        return self.direction * self.speed_value * math.pi / UNIT_180_DEG / 5

    # convert an angle in the URDF defined joint frame to an angle in the servo own frame.
    def angle_to_servo(self, angle: float) -> float:
        return self.direction * (angle - self.reference_angle)

    # convert an angle to the URDF defined joint frame from an angle in the servo own frame,
    # i.e. the inverse of `angle_to_servo`
    def angle_from_servo(self, zero_angle: float) -> float:
        return self.direction * zero_angle + self.reference_angle


# TODO(maybe): add get_angle

class Servo(Module):

    servos: Dict[str, RMServo]

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.servo
        self.node = node
        self.clock = node.get_clock()
        self.logger = node.get_logger()

        config_file = node.declare_parameter('servo.config', '').value
        if config_file:
            with open(config_file, 'r') as f:
                config = yaml.load(f, Loader=yaml.SafeLoader)
        else:
            config = []
        self.logger.info(f"Loading servos {config}")
        self.servos: Dict[int, RMServo] = {
            param['index']: RMServo(index=param['index'],
                                    reference_angle=param.get('angle', 0.0),
                                    reference_value=param.get('value', 1024),
                                    direction=param.get('direction', 1),
                                    name=param['name'])
            for param in config}
        self.servo_state_msg = sensor_msgs.msg.JointState()
        self.servo_state_msg.name = [node.tf_frame(servo.name) for servo in self.servos.values()]
        servo_rate = rate(node, 'servo', 10)
        if servo_rate:
            self.api.sub_servo_info(freq=servo_rate, callback=self.updated_servo)
            self.servo_raw_state_pub = node.create_publisher(
                robomaster_msgs.msg.ServoRawState, 'servo_raw_state', 1)
        node.create_subscription(robomaster_msgs.msg.ServoCommand, 'cmd_servo',
                                 self.has_received_cmd, 1)
        self.action: Optional[robomaster.action.Action] = None
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._move_servo_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveServo, 'move_servo', self.execute_move_servo_callback,
            goal_callback=self.new_servo_goal_callback,
            cancel_callback=self.cancel_callback, callback_group=cbg)
        get_angle_cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        node.create_service(
            robomaster_msgs.srv.GetServoAngle, 'get_servo_angle',
            self.get_servo_angle_cb, callback_group=get_angle_cbg)

    def publish_sensor_raw_state(self, msg: ServoData) -> None:
        omsg = robomaster_msgs.msg.ServoRawState()
        for i in range(4):
            omsg.valid[i] = (msg[0][i] != 0)
            omsg.speed[i] = msg[1][i]
            omsg.value[i] = msg[2][i]
        self.servo_raw_state_pub.publish(omsg)

    def stop(self) -> None:
        self._move_servo_action_server.destroy()
        if self.node.connected:
            self.api.unsub_servo_info()

    def abort(self) -> None:
        if self.action:
            self.action._abort()
            while self.action is not None:
                self.logger.info("wait for the action to terminate")
                time.sleep(0.1)

    def execute_move_servo_callback(self, goal_handle: Any
                                    ) -> robomaster_msgs.action.MoveServo.Result:
        # TODO(jerome): Complete with failures, ...
        # TODO(jerome): Check if enable concurrent actions for different servos
        request = goal_handle.request
        if request.index not in self.servos:
            self.logger.warning(f'Unknown servo with index {request.index}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveServo.Result()
        servo = self.servos[request.index]
        feedback_msg = robomaster_msgs.action.MoveServo.Feedback()
        try:
            # `angle` is the angle with respect to the servo zero in degrees
            # `index` starts from 1
            self.action = self.api.moveto(
                index=servo.index + 1, angle=round(deg(servo.angle_to_servo(request.angle))))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move servo: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveServo.Result()
        self.logger.info(f'Start moving servo with request {request}')

        def cb() -> None:
            feedback_msg.progress = cast(robomaster.action.Action, self.action)._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(self.action, cb)
        # TODO(Jerome): parametrize timeout
        self.action.wait_for_completed(timeout=7)
        if self.action.has_succeeded:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.action = None
        self.logger.info('Done moving servo')
        return robomaster_msgs.action.MoveServo.Result()

    def new_servo_goal_callback(self, goal_request: robomaster_msgs.action.MoveServo.Goal
                                ) -> rclpy.action.server.GoalResponse:
        if self.action:
            return rclpy.action.server.GoalResponse.REJECT
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        self.logger.warn('It is not possible to cancel onboard actions')
        return rclpy.action.CancelResponse.REJECT

    # (valid, speed, angle)
    def updated_servo(self, msg: ServoData) -> None:
        for v in self.servos.values():
            v.valid = (msg[0][v.index] != 0)
            if v.valid:
                v.speed_value = msg[1][v.index]
                v.value = msg[2][v.index]
        self.servo_state_msg.position = [
            servo.angle for servo in self.servos.values()]
        self.servo_state_msg.velocity = [
            servo.speed for servo in self.servos.values()]
        self.servo_state_msg.header.stamp = self.clock.now().to_msg()
        self.node.joint_state_pub.publish(self.servo_state_msg)
        for servo in self.servos.values():
            servo.valid = False
        self.publish_sensor_raw_state(msg)

    def has_received_cmd(self, msg: robomaster_msgs.msg.ServoCommand) -> None:
        if msg.index in self.servos:
            if msg.enable:
                speed = rpm(msg.angular_speed) * self.servos[msg.index].direction
                self.api.drive_speed(index=msg.index + 1, speed=speed)
                self.logger.debug(f"servo target speed set to {speed}")
            else:
                self.api.pause(index=msg.index + 1)

    def get_servo_angle_cb(self, request: robomaster_msgs.srv.GetServoAngle.Request,
                     response: robomaster_msgs.srv.GetServoAngle.Response
                     ) -> robomaster_msgs.srv.GetServoAngle.Response:
        if request.index in self.servos:
            beta: float = self.api.get_angle(index=request.index + 1)
            response.angle = self.servos[request.index].angle_from_servo(
                rad(beta) - math.pi)
        return response
