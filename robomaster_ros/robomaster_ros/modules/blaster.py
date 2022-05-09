import robomaster_msgs.msg
import robomaster.robot
import robomaster.blaster

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class Blaster(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.blaster
        self.node = node
        node.create_subscription(robomaster_msgs.msg.BlasterLED, 'blaster_led',
                                 self.has_received_blaster_led, 1)
        robot.blaster.set_led(effect=robomaster.blaster.LED_OFF)

    def stop(self) -> None:
        pass

    def abort(self) -> None:
        pass

    def has_received_blaster_led(self, msg: robomaster_msgs.msg.BlasterLED) -> None:
        if msg.brightness:
            self.api.set_led(brightness=min(255, max(0, msg.brightness * 255)),
                             effect=robomaster.blaster.LED_ON)
        else:
            self.api.set_led(effect=robomaster.blaster.LED_OFF)
