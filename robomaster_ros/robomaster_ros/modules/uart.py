import robomaster.robot
import robomaster_msgs.msg
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import nearest_index

Serial = bytearray


class Uart(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.uart
        self.node = node
        baud_rate: int = node.declare_parameter('uart.baud_rate', 115200).value
        baud_rates = [9600, 19200, 38400, 57600, 115200]
        baud_rate = nearest_index(baud_rate, baud_rates)
        # TODO(jerome) : complete
        data_bit = node.declare_parameter('uart.data_bit', 0).value
        odd_even = node.declare_parameter('uart.odd_even', 0).value
        stop_bit = node.declare_parameter('uart.stop_bit', 0).value
        rx_en = node.declare_parameter('uart.rx.enable', True).value
        tx_en = node.declare_parameter('uart.tx.enable', True).value
        rx_size = node.declare_parameter('uart.rx.size', 50).value
        tx_size = node.declare_parameter('uart.tx.size', 50).value
        self.uart_rx_pub = node.create_publisher(
            robomaster_msgs.msg.Serial, "uart/rx", 10
        )
        self.uart_tx_sub = node.create_subscription(
            robomaster_msgs.msg.Serial, "uart/tx", self.got_uart_tx, 10
        )
        self.api.serial_param_set(
            baud_rate=baud_rate,
            data_bit=data_bit,
            odd_even=odd_even,
            stop_bit=stop_bit,
            rx_en=1 if rx_en else 0,
            tx_en=1 if tx_en else 0,
            rx_size=rx_size,
            tx_size=tx_size,
        )
        self.api.start()
        self.api.sub_serial_msg(self.got_uart_rx, (), {})

    def stop(self) -> None:
        if self.node.connected:
            self.api.unsub_serial_msg()
            self.api.stop()

    def abort(self) -> None:
        pass

    def got_uart_tx(self, msg: robomaster_msgs.msg.Serial) -> None:
        self.api.serial_send_msg(bytearray(msg.data))

    def got_uart_rx(self, msg: Serial) -> None:
        self.uart_rx_pub.publish(robomaster_msgs.msg.Serial(data=list(msg)))
