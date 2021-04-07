import math
from typing import cast, List

try:
    from typing import Literal
except ImportError:
    from typing_extensions import Literal  # type: ignore


import rclpy.node


def deg(rad: float) -> float:
    return 180 / math.pi * rad


def rad(deg: float) -> float:
    return math.pi / 180 * deg


Rate = Literal[0, 1, 5, 10, 20, 50]
RATES: List[Rate] = [0, 1, 5, 10, 20, 50]


def nearest_rate(value: int) -> Rate:
    return cast(Rate, nearest(value, cast(List[int], RATES)))


def nearest_index(value: int, values: List[int]) -> int:
    ds = [abs(value - r) for r in values]
    d = min(ds)
    return ds.index(d)


def nearest(value: int, values: List[int]) -> int:
    return values[nearest_index(value, values)]


def rate(node: rclpy.node.Node, name: str, default: Rate) -> Rate:
    # DONE: add the RATE constraint
    return nearest_rate(node.declare_parameter(f"{name}.rate", default).value)
