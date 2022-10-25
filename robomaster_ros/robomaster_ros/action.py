import robomaster.action
import robomaster.robot
import types
import time
from typing import Callable, Any, Optional

import rclpy.logging


def add_cb(action: robomaster.action.Action, cb: Callable[[], None]) -> None:

    update_from_push = action.update_from_push

    def _f(self: robomaster.action.Action, proto: Any) -> None:
        update_from_push(proto)
        if proto.__class__ is not self._push_proto_cls:
            return
        cb()

    action.update_from_push = types.MethodType(_f, action)


# Action pushes updates at about 9-10 Hz.
# If they abort, they do not signal it but we can infer it from stopped updates.
# This function replaces funciton replaces both add_cb and Action.wait_for_completed
# adding a check about aborted actions.

def wait_action(action: robomaster.action.Action, cb: Callable[[], None], step: float = 0.5,
                timeout: float = 0.0) -> bool:

    if action._event.isSet() and action.is_completed:
        rclpy.logging.get_logger("SDK").info(f"Action {action} already completed")
        return True

    update_from_push = action.update_from_push

    last_update: Optional[float] = None

    def _f(self: robomaster.action.Action, proto: Any) -> None:
        nonlocal last_update
        last_update = time.time()
        update_from_push(proto)
        if proto.__class__ is not self._push_proto_cls:
            return
        cb()
        # rclpy.logging.get_logger("SDK").warning(f"Action {action} update")

    action.update_from_push = types.MethodType(_f, action)

    start = time.time()

    while True:
        last_update = None
        action._event.wait(step)
        if action._event.isSet():
            rclpy.logging.get_logger("SDK").info(f"Action {action} completed")
            return True
        if timeout > 0 and time.time() - start > timeout:
            rclpy.logging.get_logger("SDK").warning(f"Action {action} timeout")
            action._changeto_state(robomaster.action.ACTION_EXCEPTION)
            return False
        if not last_update:
            rclpy.logging.get_logger("SDK").warning(f"Action {action} stopped")
            # Aborted would be the correct keyword but is not handled by `Action.is_completed`
            action._changeto_state(robomaster.action.ACTION_FAILED)
            return False


def abort_action(robot: robomaster.robot.Robot, action: robomaster.action.Action) -> bool:
    dispatcher = robot.action_dispatcher
    key = action.make_action_key()
    if key not in dispatcher._in_progress:
        rclpy.logging.get_logger("SDK").warning("Action not in progress")
        return False
    msg = dispatcher.get_msg_by_action(action)
    # for most actions
    msg._proto._action_ctrl = 1
    # for sound playing (has no effect anyway)
    msg._proto._task_ctrl = 1
    robot.client.send_msg(msg)
    action.wait_for_completed(timeout=1.0)
    return True


def remove_action_with_target(robot: robomaster.robot.Robot, target: int) -> bool:
    rclpy.logging.get_logger("SDK").warning(f"remove_action_with_target {target}")
    dispatcher = robot.action_dispatcher
    dispatcher._in_progress_mutex.acquire()
    key: Optional[int] = None
    if dispatcher.has_in_progress_actions:
        for k, action in dispatcher._in_progress.items():
            if action.target == target:
                key = k
                break
    if key is not None:
        del dispatcher._in_progress[key]
        rclpy.logging.get_logger("SDK").warning(f"Deleted action {key}")
    dispatcher._in_progress_mutex.release()
    return key is not None
