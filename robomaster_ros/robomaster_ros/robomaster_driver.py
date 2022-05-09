from typing import Any
import time

import rclpy
import rclpy.executors
import rclpy.logging

from robomaster_ros.client import RoboMasterROS


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    should_reconnect = True
    while should_reconnect:
        node = RoboMasterROS(executor=executor)
        should_reconnect = node.reconnect
        if not node.disconnection.done():
            try:
                rclpy.spin_until_future_complete(node, node.disconnection, executor=executor)
            except KeyboardInterrupt:
                pass
        node.abort()
        rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        node.stop()
        rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        node.destroy_node()
        time.sleep(0.1)
    rclpy.shutdown()
