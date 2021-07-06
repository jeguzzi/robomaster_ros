import rclpy.publisher
import robomaster.robot
import sensor_msgs.msg

from typing import Dict, List, Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate

RangeReading = List[Tuple[int, int, int, int]]


def tof_data_info(self: robomaster.sensor.TofSubject) -> RangeReading:
    return list(zip(self._cmd_id, self._direct, self._flag, self._distance))


robomaster.sensor.TofSubject.data_info = tof_data_info


class ToF(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.sensor
        self.node = node
        self.clock = node.get_clock()
        self.range_pubs: Dict[int, rclpy.publisher.Publisher] = {}
        # TODO(jerome): tentative
        self.range_msg = sensor_msgs.msg.Range(
            radiation_type=sensor_msgs.msg.Range.INFRARED,
            min_range=0.01,
            max_range=4.0,
            field_of_view=0.1,
        )
        tof_rate = rate(node, 'tof', 10)
        if tof_rate:
            self.api.sub_distance(freq=tof_rate, callback=self.got_range_reading)

    def stop(self) -> None:
        if self.node.connected:
            self.api.unsub_distance()

    def range_pub(self, index: int) -> rclpy.publisher.Publisher:
        if index not in self.range_pubs:
            self.range_pubs[index] = self.node.create_publisher(
                sensor_msgs.msg.Range, f"range_{index}", 10
            )
        return self.range_pubs[index]

    def got_range_reading(self, readings: RangeReading) -> None:
        for i, (cmd_id, _, _, distance) in enumerate(readings):
            if cmd_id:
                msg = self.range_msg
                msg.header.frame_id = self.node.tf_frame(f'range_{i}_link')
                msg.header.stamp = self.clock.now().to_msg()
                pub = self.range_pub(i)
                pub.publish(msg)
