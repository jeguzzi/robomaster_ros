from typing import Any
import math

import rclpy
import rclpy.node
from robomaster_msgs.msg import LEDEffect
import sensor_msgs.msg
import std_msgs.msg


def color_for_charge(charge: float, intensity: float = 1.0) -> std_msgs.msg.ColorRGBA:
    if charge > 0.5:
        return std_msgs.msg.ColorRGBA(g=intensity)
    if charge > 0.25:
        return std_msgs.msg.ColorRGBA(g=intensity * 0.75, r=intensity)
    return std_msgs.msg.ColorRGBA(r=intensity)
    # return std_msgs.msg.ColorRGBA(r=intensity * (charge - 1.0), g=intensity * charge)


class BatteryDisplay(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("battery_display")
        self.create_subscription(
            sensor_msgs.msg.BatteryState, 'battery', self.has_received_data, 10)
        self.led_pub = self.create_publisher(LEDEffect, 'leds/effect', 1)
        side = self.declare_parameter('side', 'left').value
        self.mask = 32 if side == 'left' else 16
        self.count = 0

    def display(self, charge: float) -> None:
        i = min(7, math.floor(charge * 8))
        full_mask = 2 ** (i + 1) - 1
        # off_mask = 0xFF - full_mask
        self.count = (self.count + 1) % 2
        # blick 1 led
        if self.count and i == 0:
            full_mask = 0
        msg = LEDEffect(
            effect=LEDEffect.ON, mask=self.mask, color=color_for_charge(charge),
            submask=full_mask)
        self.led_pub.publish(msg)
        # if i < 7:
        #     self.led_pub.publish(
        #         LEDEffect(effect=LEDEffect.ON, mask=self.mask, submask=off_mask,
        #                   color=std_msgs.msg.ColorRGBA()))

    def has_received_data(self, msg: sensor_msgs.msg.BatteryState) -> None:
        self.display(msg.percentage)

    def stop(self) -> None:
        self.led_pub.publish(LEDEffect(effect=LEDEffect.OFF))


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = BatteryDisplay()
    rclpy.spin(node)
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
