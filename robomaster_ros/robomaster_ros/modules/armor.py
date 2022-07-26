import robomaster.robot
import robomaster_msgs.msg

from typing import Tuple, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

import rcl_interfaces.msg

from .. import Module


ArmorHit = Tuple[int, int, int]


def hit_data_info(self: robomaster.armor.ArmorHitEvent) -> ArmorHit:
    return (self._armor_id, self._type, self._mic_value)


robomaster.armor.ArmorHitEvent.data_info = hit_data_info


sensitivity_param_desc = """The relative detection threshold between 0 and 1 (in 0.1 steps)
for all four collision sensors. Lower the value to increase sensitivity."""


class Armor(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.armor
        self.clock = node.get_clock()
        self.hit_pub = node.create_publisher(robomaster_msgs.msg.ArmorHit, 'hit', 10)
        desc = rcl_interfaces.msg.ParameterDescriptor(
            description=sensitivity_param_desc,
            floating_point_range=[
                rcl_interfaces.msg.FloatingPointRange(from_value=0.0, to_value=1.0, step=0.1)])
        sensitivity = node.declare_parameter('armor.sensitivity', 0.5, descriptor=desc).value
        self.set_sensitivity(sensitivity)
        self.api.sub_hit_event(self.has_been_hit)
        node.add_on_set_parameters_callback(self.set_params_cb)

    def set_sensitivity(self, value: float) -> None:
        rm_sensitivity = max(0, min(10, int(10 * value)))
        self.api.set_hit_sensitivity(sensitivity=rm_sensitivity)

    def set_params_cb(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        for param in params:
            if param.name == 'armor.sensitivity':
                if param.value > 1:
                    param.value = 1.0
                if param.value < 0:
                    param.valuye = 0.0
                self.set_sensitivity(param.value)

        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def stop(self) -> None:
        pass

    def abort(self) -> None:
        pass

    def has_been_hit(self, msg: ArmorHit) -> None:
        ros_msg = robomaster_msgs.msg.ArmorHit(location=msg[0], type=msg[1], level=msg[2])
        ros_msg.header.stamp = self.clock.now().to_msg()
        self.hit_pub.publish(ros_msg)
