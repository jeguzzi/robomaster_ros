import math
import yaml
import time

import rclpy.action

import robomaster.robot
import sensor_msgs.msg
import robomaster_msgs.msg
import robomaster_msgs.action

from typing import Dict, List, Tuple, Any, Optional
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


def rpm(angular_speed: float) -> float:
    return max(min(MAX_RPM, 30 * angular_speed / math.pi), -MAX_RPM)


class RMServo:
    index: int
    valid: bool
    zero: float
    value: int
    speed_value: int
    name: str

    def __init__(self, index: int, reference_angle: float, reference_value: int, slope: float,
                 name: str = '') -> None:
        self.index = index
        self.valid = False
        self.rad2servo = slope
        # if direction > 0:
        #     self.direction = 1
        # else:
        #     self.direction = -1
        # self.bias = reference_angle * self.rad2servo + reference_value
        self.reference_angle = reference_angle
        self.reference_value = reference_value
        self.external_reference_angle = reference_value / self.rad2servo
        self.name = name

    @property
    def angle(self) -> float:
        return self.reference_angle + (self.value - self.reference_value) / self.rad2servo

    @property
    def speed(self) -> float:
        return self.speed_value / self.rad2servo / 5

    # TODO(Jerome): external now invalid with new calibration
    def external(self, angle: float) -> float:
        return 0
        # return self.direction * (angle - self.reference_angle) + self.external_reference_angle


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
            param['index']: RMServo(index=param['index'], reference_angle=param['angle'],
                                    reference_value=param['value'], slope=param['slope'],
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
        self._move_servo_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveServo, 'move_servo', self.execute_move_servo_callback,
            cancel_callback=self.cancel_callback)

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
        request = goal_handle.request
        servo = self.servos[request.index]
        feedback_msg = robomaster_msgs.action.MoveServo.Feedback()
        try:
            self.action = self.api.moveto(
                index=servo.index + 1, angle=round(deg(servo.external(request.angle))))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move servo: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveServo.Result()
        self.logger.info(f'Start moving servo with request {request}')

        def cb() -> None:
            feedback_msg.progress = self.action._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(self.action, cb)
        self.action.wait_for_completed()
        if self.action.has_succeeded:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.action = None
        self.logger.info('Done moving servo')
        return robomaster_msgs.action.MoveServo.Result()

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
        if msg.enable:
            # self.logger.info("\n\nSEND drive_speed\n\n")
            speed = rpm(msg.angular_speed)
            self.api.drive_speed(index=msg.index + 1, speed=speed)
            self.logger.info(f"SENT drive_speed {speed}")
        else:
            self.api.pause()
