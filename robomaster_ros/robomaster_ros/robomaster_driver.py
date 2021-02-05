from typing import Any

import rclpy

from robomaster_ros.client import RoboMasterROS


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = RoboMasterROS()
    try:
        # print(node.disconnection)
        rclpy.spin_until_future_complete(node, node.disconnection)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
