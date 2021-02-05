import robomaster.robot
import sensor_msgs.msg

from typing import Dict, List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate

# DONE form the APP 1 unit = 0.17578 degress
RAD2SERVO = 325.95

ServoData = Tuple[List[int], List[int], List[int]]


class RMServo:
    index_: int
    valid: bool
    zero: float
    value: int
    speed_value: int

    def __init__(self, index: int, reference_angle: float, reference_value: int) -> None:
        self.index_ = index
        self.valid = False
        self.bias = reference_angle * RAD2SERVO + reference_value

    @property
    def angle(self) -> float:
        return (self.bias - self.value) / RAD2SERVO

    @property
    def speed(self) -> float:
        return self.speed_value / RAD2SERVO


class Servo(Module):

    servos: Dict[str, RMServo]

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.servo
        self.node = node
        self.servo_state_msg = sensor_msgs.msg.JointState()
        self.clock = node.get_clock()
        servo_rate = rate(node, 'servo', 10)
        # TODO(Jerome) : complete
        self.servos = {}
        if servo_rate:
            self.api.sub_servo_info(freq=servo_rate, callback=self.updated_servo)

    def stop(self) -> None:
        self.api.unsub_servo_info()

    # (valid, speed, angle)
    def updated_servo(self, msg: ServoData) -> None:
        for v in self.servos.values():
            v.valid = (msg[0][v.index_] != 0)
            if v.valid:
                v.speed_value = msg[1][v.index_]
                v.value = msg[2][v.index_]
        self.servo_state_msg.name = [name for name, servo in self.servos.items() if servo.valid]
        self.servo_state_msg.position = [
            servo.angle for servo in self.servos.values() if servo.valid]
        self.servo_state_msg.velocity = [
            servo.speed for servo in self.servos.values() if servo.valid]
        self.servo_state_msg.header.stamp = self.clock.now().to_msg()
        self.node.joint_state_pub.publish(self.servo_state_msg)
        for servo in self.servos.values():
            servo.valid = False
