import os
import time

import rclpy.action
import robomaster.robot
import robomaster_msgs.msg

from ..action import add_cb

from typing import Any, Dict, Optional
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class Speaker(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot
        self.logger = node.get_logger()
        self.action: Optional[robomaster.action.Action] = None
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sound_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.PlaySound, "play", self.execute_play_sound,
            cancel_callback=self.cancel_callback, callback_group=cbg)
        self._audio_files: Dict[int, str] = {}
        self._audio_id = 0

    # Using this version as it caches the files/sound_id
    def load_audio(self, filename: str) -> int:
        filename = os.path.abspath(filename)
        for audio_id, name in self._audio_files.items():
            if filename == name:
                return 0xE0 + audio_id
        if not os.path.exists(filename):
            self.logger.warning(f"Loading audio: file {filename} does not exists")
            return -1
        if self.api._ftp.connected:
            self._audio_id = (self._audio_id + 1) % 10
            self._audio_files[self._audio_id] = filename
            upload_file = f"/python/sdk_audio_{self._audio_id}.wav"
            self.logger.info(f"Loading audio: file {filename} as sound {0xE0 + self._audio_id}")
            self.api._ftp.upload(filename, upload_file)
            return 0xE0 + self._audio_id
        else:
            return -1

    def stop(self) -> None:
        self._sound_action_server.destroy()

    def abort(self) -> None:
        if self.action:
            self.action._abort()
            while self.action is not None:
                self.logger.info("wait for the action to terminate")
                time.sleep(0.1)

    def execute_play_sound(self, goal_handle: Any) -> robomaster_msgs.action.PlaySound.Result:
        request = goal_handle.request
        if request.file:
            sound_id = self.load_audio(request.file)
        else:
            sound_id = request.sound_id
        if sound_id < 0:
            goal_handle.abort()
            self.logger.warning('Cannot play sound')
            return robomaster_msgs.action.PlaySound.Result()
        try:
            self.action = self.api.play_sound(sound_id=sound_id, times=request.times)
        except RuntimeError as e:
            self.logger.warning(f'Cannot play sound: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.PlaySound.Result()
        self.logger.info(f"Start playing sound with request {request}")
        feedback_msg = robomaster_msgs.action.PlaySound.Feedback()
        feedback_msg.time = 0
        progress = [2.0]

        def cb() -> None:
            feedback_msg.progress = self.action._percent * 0.01
            if feedback_msg.progress < progress[0]:
                feedback_msg.time += 1
            progress[0] = feedback_msg.progress
            goal_handle.publish_feedback(feedback_msg)

        add_cb(self.action, cb)
        self.action.wait_for_completed()
        if self.action.has_succeeded:
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        self.action = None
        self.logger.info("Done playing sound")
        return robomaster_msgs.action.PlaySound.Result()

    def cancel_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        self.logger.warn('It is not possible to cancel onboard actions')
        return rclpy.action.CancelResponse.REJECT
