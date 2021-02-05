import robomaster.action
import types
from typing import Callable, Any


def add_cb(action: robomaster.action.Action, cb: Callable[[], None]) -> None:

    update_from_push = action.update_from_push

    def _f(self: robomaster.action.Action, proto: Any) -> None:
        update_from_push(proto)
        if proto.__class__ is not self._push_proto_cls:
            return
        cb()

    action.update_from_push = types.MethodType(_f, action)
