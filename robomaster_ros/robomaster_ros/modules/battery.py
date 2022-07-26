import sensor_msgs.msg
from sensor_msgs.msg import BatteryState

import robomaster.battery
from typing import Tuple
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module

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
        self.battery_state_msg.design_capacity = 2.4
        self.battery_state_msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        self.battery_state_msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        self.battery_state_msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

        self.battery_pub = node.create_publisher(sensor_msgs.msg.BatteryState, 'battery', 1)
        self.api = robot.battery
        self.node = node
        self.clock = node.get_clock()
        self.api.sub_battery_info(freq=1, callback=self.updated_battery)

    def stop(self) -> None:
        if self.node.connected:
            self.api.unsub_battery_info()

    def abort(self) -> None:
        pass

    def updated_battery(self, msg: BatteryData) -> None:
        self.battery_state_msg.header.stamp = self.clock.now().to_msg()
        self.battery_state_msg.voltage = float(0.001 * msg[0])
        self.battery_state_msg.current = float(0.001 * msg[2])
        self.battery_state_msg.percentage = msg[3] * 0.01
        self.battery_state_msg.temperature = msg[1] * 0.1
        self.battery_pub.publish(self.battery_state_msg)
