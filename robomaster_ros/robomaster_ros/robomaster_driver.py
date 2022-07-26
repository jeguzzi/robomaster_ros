from typing import Any
import time

import rclpy
import rclpy.executors
import rclpy.logging

from robomaster_ros.client import RoboMasterROS


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    # TODO(Jerome): currently not triggered by ctrl+C
    # rclpy.get_default_context().on_shutdown(...)
    should_reconnect = True
    while should_reconnect:
        try:
            node = RoboMasterROS(executor=executor)
        except KeyboardInterrupt:
            break
        should_reconnect = node.reconnect
        if not node.disconnection.done():
            try:
                rclpy.spin_until_future_complete(node, node.disconnection, executor=executor)
            except KeyboardInterrupt:
                node.get_logger().warn('KeyboardInterrupt')
                should_reconnect = False
        node.abort()
        if rclpy.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        node.stop()
        if rclpy.ok():
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        node.destroy_node()
        time.sleep(0.1)
    if rclpy.ok():
        # TODO(Jerome): maybe remove, as probably never needed
        rclpy.shutdown()
