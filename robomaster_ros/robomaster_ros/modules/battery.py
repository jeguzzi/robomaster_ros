import sensor_msgs.msg

import robomaster.battery
from typing import Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module

# TODO measure
VOLTAGE_F = 0.00035
# TODO measure
CURRENT_F = 0.0035


BatteryData = Tuple[int, int, int, int]


def battery_data_info(self: robomaster.battery.BatterySubject) -> BatteryData:
    return (self._adc_value, self._temperature, self._current, self._percent)


robomaster.battery.BatterySubject.data_info = battery_data_info


class Battery(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.battery_state_msg = sensor_msgs.msg.BatteryState()
        self.battery_state_msg.present = True
        self.battery_state_msg.charge = -1.0
        self.battery_state_msg.capacity = -1.0
        self.battery_state_msg.design_capacity = 6.2
        self.battery_pub = node.create_publisher(sensor_msgs.msg.BatteryState, 'battery', 1)
        self.api = robot.battery
        self.node = node
        self.clock = node.get_clock()
        self.api.sub_battery_info(freq=1, callback=self.updated_battery)

    def stop(self) -> None:
        if self.node.connected:
            self.api.unsub_battery_info()

    def updated_battery(self, msg: BatteryData) -> None:
        self.battery_state_msg.header.stamp = self.clock.now().to_msg()
        self.battery_state_msg.voltage = float(VOLTAGE_F * msg[0])
        self.battery_state_msg.current = float(CURRENT_F * msg[2])
        self.battery_state_msg.percentage = msg[3] * 0.01
        self.battery_pub.publish(self.battery_state_msg)
