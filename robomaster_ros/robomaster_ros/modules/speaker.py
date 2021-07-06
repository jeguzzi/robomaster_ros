import os

import rclpy.action
import robomaster.robot
import robomaster_msgs.msg

from ..action import add_cb

from typing import Any, Dict
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
            action = self.api.play_sound(sound_id=sound_id, times=request.times)
        except RuntimeError as e:
            self.logger.warning(f'Cannot play sound: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.PlaySound.Result()
        self.logger.info(f"Start playing sound with request {request}")
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
        self.logger.info("Done playing sound")
        return robomaster_msgs.action.PlaySound.Result()
