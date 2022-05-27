# Adapted SDK examples/01_robot/05_sta_conn_helper.py
import argparse
from typing import Any

import qrcode
import robomaster.conn
import rclpy.utilities
import rclpy.logging


def main(args: Any = None) -> None:
    args = rclpy.utilities.remove_ros_args(args=args)
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('ssid', type=str, help='The network SSID')
    parser.add_argument('password', type=str, help='The network password')
    parser.add_argument('--app_id', default='', type=str, help='The app id of at most 7 letters')
    args = parser.parse_args(args[1:])
    helper = robomaster.conn.ConnectionHelper()
    if args.app_id:
        helper._appid = args.app_id[:8]
    info = helper.build_qrcode_string(ssid=args.ssid, password=args.password)
    img = qrcode.make(info)
    img.show()
    logger = rclpy.logging.get_logger("connect")
    if helper.wait_for_connection():
        logger.info("Connected")
    else:
        logger.error("Connection failed")
    # TODO(Jerome): close the image display
    # As of now, it will leave the window open because `img.show`
    # spawns a separate process to display an image
