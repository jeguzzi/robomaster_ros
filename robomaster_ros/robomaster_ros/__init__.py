from abc import ABC
import robomaster.robot

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS


class Module(ABC):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        ...

    def stop(self) -> None:
        ...
