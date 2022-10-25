import os
import time

import rclpy.action
import robomaster.robot
import robomaster.protocol
import robomaster_msgs.msg

from ..action import wait_action

from typing import Any, Dict, Optional, cast
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


# Extends RobotPlaySoundAction to support specifying play_ctr:
# 0: stops playing current sound (if sound_id corresponds)
# 1: stops playing current sound (if any) and play sound
# 2: plays sound in parallel with current sound
# 3: does nothing
class ExtendedPlaySoundAction(robomaster.robot.RobotPlaySoundAction):  # type: ignore
    _action_proto_cls = robomaster.protocol.ProtoPlaySound
    _push_proto_cls = robomaster.protocol.ProtoSoundPush
    _target = robomaster.protocol.host2byte(9, 0)

    def __init__(self, sound_id: int, times: int, ctr: int = 1,
                 **kw: Any) -> None:
        super().__init__(sound_id, times, **kw)
        self._play_ctrl = ctr

    def encode(self) -> robomaster.protocol.ProtoPlaySound:
        proto = super().encode()
        proto._play_ctrl = self._play_ctrl
        return proto


robomaster.robot.RobotPlaySoundAction = ExtendedPlaySoundAction


class Speaker(Module):
    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot
        self.node = node
        self.logger = node.get_logger()
        node.create_subscription(robomaster_msgs.msg.SpeakerCommand, 'cmd_sound',
                                 self.has_received_play_command, 10)
        self.action: Optional[robomaster.action.Action] = None
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self._sound_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.PlaySound, "play", self.execute_play_sound,
            goal_callback=self.new_speaker_goal_callback,
            cancel_callback=self.cancel_callback, callback_group=cbg)
        self._audio_files: Dict[int, str] = {}
        self._audio_id = 0

    def play_action(self, sound_id: int, times: int = 1, control: int = 1
                    ) -> ExtendedPlaySoundAction:
        action = ExtendedPlaySoundAction(sound_id, times, control)
        self.api._action_dispatcher.send_action(action)
        return action

    def play(self, sound_id: int, times: int = 1, control: int = 1, interval: int = 0) -> None:
        action = ExtendedPlaySoundAction(sound_id, times, control)
        action._action_id = action._get_next_action_id()
        msg = self.api._action_dispatcher.get_msg_by_action(action)
        self.logger.info(f"play_ctrl: {msg._proto._play_ctrl}, action._play_ctrl")
        self.api.client.send_msg(msg)

    def has_received_play_command(self, msg: robomaster_msgs.msg.SpeakerCommand) -> None:
        self.play(sound_id=msg.sound_id, times=msg.times, control=msg.control)

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
        progress = 2.0

        def cb() -> None:
            nonlocal progress
            feedback_msg.progress = cast(robomaster.action.Action, self.action)._percent * 0.01
            if feedback_msg.progress < progress:
                feedback_msg.time += 1
            progress = feedback_msg.progress
            goal_handle.publish_feedback(feedback_msg)

        wait_action(self.action, cb)
        # TODO(Jerome): relax
        # self.action.wait_for_completed(timeout=10)
        if self.action.has_succeeded:
            self.logger.info('Done playing sound: succeed')
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            self.logger.info('Done playing sound: canceled')
            goal_handle.canceled()
        else:
            self.logger.warning('Done playing sound: aborted')
            try:
                goal_handle.abort()
            except rclpy._rclpy_pybind11.RCLError:
                # Happens when the context shutdown before we send feedbacks
                pass
        self.action = None
        return robomaster_msgs.action.PlaySound.Result()

    def new_speaker_goal_callback(self, goal_request: robomaster_msgs.action.PlaySound.Goal
                                  ) -> rclpy.action.server.GoalResponse:
        if self.action:
            return rclpy.action.server.GoalResponse.REJECT
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        # self.logger.warn('It is not possible to cancel onboard actions')
        # return rclpy.action.CancelResponse.REJECT
        if self.action:
            self.node.executor.create_task(self._stop_action)
        return rclpy.action.CancelResponse.ACCEPT

    def _stop_action(self) -> None:
        if self.action:
            self.logger.info('Canceling play sound action')
            sound_id = self.action._sound_id
            self.action._changeto_state(robomaster.action.ACTION_FAILED)
            action = self.play_action(sound_id=sound_id, control=0)
            action.wait_for_completed(timeout=1.0)
