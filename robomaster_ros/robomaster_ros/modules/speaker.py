import rclpy.action
import robomaster.robot
import robomaster_msgs.msg

from ..action import add_cb

from typing import Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class Speaker(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot
        self.logger = node.get_logger()
        self._sound_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.PlaySound, "play", self.execute_play_sound
        )

    def execute_play_sound(self, goal_handle: Any) -> robomaster_msgs.action.PlaySound.Result:
        self.logger.info("[Action begin] play sound")
        request = goal_handle.request
        action = self.api.play_sound(sound_id=request.sound_id, times=request.times)
        feedback_msg = robomaster_msgs.action.PlaySound.Feedback()
        feedback_msg.time = 0
        progress = [2.0]

        def cb() -> None:
            feedback_msg.progress = action._percent * 0.01
            if feedback_msg.progress < progress[0]:
                feedback_msg.time += 1
            progress[0] = feedback_msg.progress
            goal_handle.publish_feedback(feedback_msg)

        add_cb(action, cb)
        action.wait_for_completed()
        goal_handle.succeed()
        return robomaster_msgs.action.PlaySound.Result()
