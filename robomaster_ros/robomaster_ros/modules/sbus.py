import robomaster_msgs.msg

import robomaster.robot

from typing import List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate

SBusData = Tuple[int, List[int]]


class SBus(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.clock = node.get_clock()
        self.logger = node.get_logger()
        self.node = node
        self.api = robot.chassis
        self.sbus_pub = node.create_publisher(robomaster_msgs.msg.SBus, 'sbus', 1)
        sbus_rate = rate(node, 'sbus', 0)
        if sbus_rate:
            self.api._sub_sbus(freq=sbus_rate, callback=self.updated_sbus)

    def stop(self) -> None:
        if self.node.connected:
            self.api._unsub_sbus()

    def abort(self) -> None:
        pass

    # (connection, channels)
    def updated_sbus(self, msg: SBusData) -> None:
        if msg[0]:
            ros_msg = robomaster_msgs.msg.SBus(channels=msg[1])
            ros_msg.header.stamp = self.clock.now().to_msg()
            self.sbus_pub.publish(ros_msg)
