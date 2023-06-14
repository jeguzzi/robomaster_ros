# Adapted SDK examples/01_robot/05_sta_conn_helper.py
import argparse
import time
import logging
import socket
from typing import Any, Dict, Optional

import rclpy.utilities
import rclpy.logging

import robomaster
import robomaster.config
import robomaster.robot
import robomaster.conn
import robomaster.led


from robomaster_ros.ftp import FtpConnection


def discover_robots(timeout: float = 3.0) -> Dict[str, str]:
    # ip -> sn
    robots: Dict[str, str] = {}
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", robomaster.config.ROBOT_BROADCAST_PORT))
    if timeout > 0:
        s.settimeout(timeout)
    start = time.time()
    while time.time() - start < timeout:
        try:
            data, (ip, port) = s.recvfrom(1024)
        except socket.timeout:
            continue
        if ip not in robots:
            sn = str(data[:-1].decode(encoding='utf-8'))
            robots[ip] = sn
    return robots


def probe_robot(ip: str, sn: str, logger: logging.Logger,
                modules: bool = True) -> bool:
    ep_robot = robomaster.robot.Robot()
    try:
        ep_robot.initialize(conn_type="sta", sn=sn)
    except (AttributeError, TypeError):
        logger.error(f"Could not connect to robot with sn {sn}")
        return False
    version = ep_robot.get_version()
    if not version:
        logger.error(f"Could not get version of robot with sn {sn}")
        return False

    battery_level: Optional[float] = None

    def battery_cb(msg: float) -> None:
        nonlocal battery_level
        battery_level = msg

    ep_robot.battery.sub_battery_info(0, battery_cb)

    while battery_level is None:
        time.sleep(0.1)

    logger.info(f'Connected to robot {sn} at {ip}: '
                f'version {version}, battery {battery_level:d}%')
    play = ep_robot.play_sound(7, times=1)
    ep_robot.led.set_led(r=255, effect=robomaster.led.EFFECT_FLASH, freq=5)
    if modules:
        for name, module in sorted(ep_robot._modules.items()):
            logger.info(f'\t{name} ?')
            version = module.get_version()
            if version:
                logger.info(f'\t{name} {version}')
        else:
            time.sleep(1)
    play.wait_for_completed()
    ep_robot.led.set_led(effect=robomaster.led.EFFECT_OFF)
    ep_robot.play_sound(8, times=1).wait_for_completed()
    ep_robot.close()
    return True


def main(args: Any = None) -> None:
    args = rclpy.utilities.remove_ros_args(args=args)
    logger = rclpy.logging.get_logger("discover")
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('--timeout', type=float, default=3.0, help='How much to wait for robots')
    parser.add_argument('--modules', type=bool, default=False, help='Probe the robot modules')
    args = parser.parse_args(args[1:])
    robomaster.logger.setLevel(logging.CRITICAL)
    robots = discover_robots(timeout=args.timeout)
    logger.info(f'Discovered {len(robots)} robots')
    robomaster.conn.FtpConnection = FtpConnection
    for ip, sn in robots.items():
        probe_robot(ip, sn, logger=logger, modules=args.modules)
