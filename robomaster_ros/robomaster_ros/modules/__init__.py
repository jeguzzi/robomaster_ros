from typing import Dict, Type
from .. import Module

from .arm import Arm  # noqa
from .armor import Armor  # noqa
from .battery import Battery  # noqa
from .camera import Camera  # noqa
from .chassis import Chassis  # noqa
from .gripper import Gripper  # noqa
from .led import LED  # noqa
from .pwm import PWM  # noqa
from .sbus import SBus  # noqa
from .servo import Servo  # noqa
from .speaker import Speaker  # noqa
from .tof import ToF  # noqa
from .uart import Uart  # noqa
from .vision import Vision  # noqa
from .gimbal import Gimbal  # noqa
from .blaster import Blaster  # noqa
from .sensor_adapter import SensorAdapter  # noqa

modules: Dict[str, Type[Module]] = {
    'arm': Arm,
    'armor': Armor,
    'battery': Battery,
    'camera': Camera,
    'chassis': Chassis,
    'gripper': Gripper,
    'led': LED,
    'pwm': PWM,
    'sbus': SBus,
    'servo': Servo,
    'speaker': Speaker,
    'tof': ToF,
    'uart': Uart,
    'vision': Vision,
    'gimbal': Gimbal,
    'blaster': Blaster,
    'sensor_adapter': SensorAdapter
}
