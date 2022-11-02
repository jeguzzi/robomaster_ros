from typing import Any, Optional
import time
import signal

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

    node: Optional[RoboMasterROS] = None

    def shutdown(sig, _):
        nonlocal should_reconnect
        should_reconnect = False
        if node:
            if rclpy.ok():
                node.abort()
                rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
            if rclpy.ok():
                node.stop()
                rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        else:
            raise KeyboardInterrupt
        rclpy.try_shutdown()

    signal.signal(signal.SIGINT, shutdown)

    while should_reconnect and rclpy.ok():
        try:
            node = RoboMasterROS(executor=executor)
        except KeyboardInterrupt:
            break
        should_reconnect = node.reconnect
        if not node.disconnection.done():
            try:
                while rclpy.ok() and not node.disconnection.done():
                    rclpy.spin_until_future_complete(
                        node, node.disconnection, executor=executor, timeout_sec=1.0)
            except KeyboardInterrupt:
                should_reconnect = False
                break
            except rclpy._rclpy_pybind11.RCLError:
                should_reconnect = False
                break
        if rclpy.ok():
            node.abort()
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        if rclpy.ok():
            node.stop()
            rclpy.spin_once(node, executor=executor, timeout_sec=0.1)
        node.destroy_node()
        node = None
        time.sleep(0.1)
    rclpy.try_shutdown()
    if node:
        node.destroy_node()
