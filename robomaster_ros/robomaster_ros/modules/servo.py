import math
import yaml

import rclpy.action

import robomaster.robot
import sensor_msgs.msg
import robomaster_msgs.msg
import robomaster_msgs.action

from typing import Dict, List, Tuple, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate, deg
from ..action import add_cb

# DONE form the APP 1 unit = 0.17578 degress
RAD2SERVO = 325.95
MAX_RPM = 49

ServoData = Tuple[List[int], List[int], List[int]]


def rpm(angular_speed: float) -> int:
    return max(min(MAX_RPM, round(30 * angular_speed / math.pi)), -MAX_RPM)


class RMServo:
    index: int
    valid: bool
    zero: float
    value: int
    speed_value: int
    name: str

    def __init__(self, index: int, reference_angle: float, reference_value: int,
                 direction: int, name: str = '') -> None:
        self.index = index
        self.valid = False
        if direction > 0:
            self.direction = 1
        else:
            self.direction = -1
        self.bias = reference_angle * RAD2SERVO + reference_value
        self.reference_angle = reference_angle
        self.external_reference_angle = reference_value / RAD2SERVO
        self.name = name

    @property
    def angle(self) -> float:
        return self.direction * (self.value - self.bias) / RAD2SERVO

    @property
    def speed(self) -> float:
        return self.direction * self.speed_value / RAD2SERVO / 5

    def external(self, angle: float) -> float:
        return self.direction * (angle - self.reference_angle) + self.external_reference_angle


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
                config = yaml.load(f)
        else:
            config = []
        self.logger.info(f"Loading servos {config}")
        self.servos: Dict[int, RMServo] = {
            param['index']: RMServo(index=param['index'], reference_angle=param['angle'],
                                    reference_value=param['value'], direction=param['direction'],
                                    name=param['name'])
            for param in config}
        self.servo_state_msg = sensor_msgs.msg.JointState()
        self.servo_state_msg.name = [node.tf_frame(servo.name) for servo in self.servos.values()]
        servo_rate = rate(node, 'servo', 10)
        if servo_rate:
            self.api.sub_servo_info(freq=servo_rate, callback=self.updated_servo)
        node.create_subscription(robomaster_msgs.msg.ServoCommand, 'cmd_servo',
                                 self.has_received_cmd, 1)
        self._move_servo_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveServo, 'move_servo', self.execute_move_servo_callback)

    def stop(self) -> None:
        self._move_servo_action_server.destroy()
        if self.node.connected:
            self.api.unsub_servo_info()

    def execute_move_servo_callback(self, goal_handle: Any
                                    ) -> robomaster_msgs.action.MoveServo.Result:
        # TODO(jerome): Complete with failures, ...
        request = goal_handle.request
        servo = self.servos[request.index]
        feedback_msg = robomaster_msgs.action.MoveServo.Feedback()
        try:
            action = self.api.moveto(
                index=servo.index + 1, angle=round(deg(servo.external(request.angle))))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move servo: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveServo.Result()
        self.logger.info('Start moving servo with request {request}')

        def cb() -> None:
            feedback_msg.progress = action._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(action, cb)
        action.wait_for_completed()
        goal_handle.succeed()
        self.logger.info('Done moving servo')
        return robomaster_msgs.action.MoveServo.Result()

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

    def has_received_cmd(self, msg: robomaster_msgs.msg.ServoCommand) -> None:
        if msg.enable:
            self.logger.info("\n\nSEND drive_speed\n\n")
            self.api.drive_speed(index=msg.index + 1, speed=rpm(msg.angular_speed))
            self.logger.info("\n\nSENT drive_speed\n\n")
        else:
            self.api.pause()
