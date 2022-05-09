import robomaster_msgs.msg

import robomaster.robot

from typing import List
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class PWM(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        pwm_frequency: List[int] = node.declare_parameter(
            "pwm.frequencies", [-1, -1, -1, -1, -1, -1]).value
        pwm_frequency_values = [None if v < 0 else v for v in pwm_frequency[:6]]
        if len(pwm_frequency_values) < 6:
            pwm_frequency_values += [None] * (6 - len(pwm_frequency_values))
        self.api = robot.chassis
        node.get_logger().info(f"Setting PWM frequencies {pwm_frequency_values}")
        self.api.set_pwm_freq(*pwm_frequency_values)
        node.create_subscription(
            robomaster_msgs.msg.PWM, 'cmd_pwm', self.has_received_pwm, 1)

    def stop(self) -> None:
        pass

    def abort(self) -> None:
        pass

    def has_received_pwm(self, msg: robomaster_msgs.msg.PWM) -> None:
        self.api.set_pwm_value(
            *[None if v < 0 else min(100, max(0, 100 * v)) for v in msg.fraction_of_duty_cycle])
