import robomaster.robot
import robomaster_msgs.msg

from typing import Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


ArmorHit = Tuple[int, int, int]


def hit_data_info(self: robomaster.armor.ArmorHitEvent) -> ArmorHit:
    return (self._armor_id, self._type, self._mic_value)


robomaster.armor.ArmorHitEvent.data_info = hit_data_info


class Armor(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.armor
        self.hit_pub = node.create_publisher(robomaster_msgs.msg.ArmorHit, 'hit', 10)
        sensitivity = node.declare_parameter('armor.sensitivity', 0.5).value
        rm_sensitivity = max(0, min(10, int(10 * sensitivity)))
        self.api.set_hit_sensitivity(sensitivity=rm_sensitivity)
        self.api.sub_hit_event(self.has_been_hit)

    def stop(self) -> None:
        pass

    def has_been_hit(self, msg: ArmorHit) -> None:
        ros_msg = robomaster_msgs.msg.ArmorHit(location=msg[0], type=msg[1], level=msg[2])
        self.hit_pub.publish(ros_msg)
