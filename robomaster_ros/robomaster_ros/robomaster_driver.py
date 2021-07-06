from typing import Any
import time

import rclpy
import rclpy.executors
import rclpy.logging

from robomaster_ros.client import RoboMasterROS


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = RoboMasterROS(executor=executor)
    if not node.disconnection.done():
        try:
            rclpy.spin_until_future_complete(node, node.disconnection, executor=executor)
        except KeyboardInterrupt:
            pass
    time.sleep(0.1)
    node.stop()
    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
