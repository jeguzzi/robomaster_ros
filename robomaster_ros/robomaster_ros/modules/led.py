import std_msgs.msg
import robomaster_msgs.msg

import robomaster.robot
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


def channel(value: float) -> int:
    return min(max(0, round(255 * value)), 255)


class LED(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.led
        self.node = node
        self.api.set_led(comp=robomaster.led.COMP_ALL, effect=robomaster.led.EFFECT_OFF,
                         r=255, g=255, b=0)
        node.create_subscription(
            robomaster_msgs.msg.LEDEffect, 'leds/effect', self.has_received_led_effect, 1)
        node.create_subscription(
            std_msgs.msg.ColorRGBA, 'leds/color', self.has_received_led_color, 1)

    def stop(self) -> None:
        if self.node.connected:
            self.api.set_led(comp=robomaster.led.COMP_ALL, effect=robomaster.led.EFFECT_OFF)

    def abort(self) -> None:
        pass

    def has_received_led_effect(self, msg: robomaster_msgs.msg.LEDEffect) -> None:
        proto = robomaster.protocol.ProtoSetSystemLed()
        proto._ctrl_mode = 7
        proto._comp_mask = msg.mask
        proto._led_mask = msg.submask
        color = (msg.color.r, msg.color.g, msg.color.b)
        (proto._r, proto._g, proto._b) = [
            robomaster.util.COLOR_VALUE_CHECKER.val2proto(channel(c)) for c in color]
        proto._effect_mode = msg.effect
        proto._t1, proto._t2 = [round(value * 1000) for value in (msg.t1, msg.t2)]
        return self.api._send_sync_proto(proto, robomaster.protocol.host2byte(9, 0))

    def has_received_led_color(self, msg: std_msgs.msg.ColorRGBA) -> None:
        if msg.r == msg.g == msg.b == 0:
            effect = robomaster.led.EFFECT_OFF
        else:
            effect = robomaster.led.EFFECT_ON
        self.api.set_led(
            comp=robomaster.led.COMP_ALL, effect=effect, r=channel(msg.r), g=channel(msg.g),
            b=channel(msg.b))
