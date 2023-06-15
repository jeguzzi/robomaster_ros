import time

import rclpy.action
import robomaster.robot
import robomaster.action

import robomaster_msgs.action
import robomaster.gimbal
import robomaster.robot
import robomaster_msgs.msg
import sensor_msgs.msg
import std_srvs.srv
import std_msgs.msg
import rcl_interfaces.msg

import rclpy.logging

from typing import Tuple, Any, Optional, cast
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import wait_action
from ..utils import rate, nearest_rate, rad, deg, Rate

GimbalData = Tuple[float, float, float, float]


def mode2api(mode: int) -> str:
    if mode == 0:
        return robomaster.robot.FREE
    if mode == 1:
        return robomaster.robot.GIMBAL_LEAD
    return robomaster.robot.CHASSIS_LEAD


def move(api: robomaster.gimbal.Gimbal, pitch: float = 0, yaw: float = 0, pitch_speed: float = 30,
         yaw_speed: float = 0, frame: int = robomaster.gimbal.COORDINATE_CUR
         ) -> robomaster.gimbal.GimbalMoveAction:
    if frame == robomaster.gimbal.COORDINATE_CUR:
        pitch = robomaster.util.GIMBAL_PITCH_MOVE_CHECKER.val2proto(pitch)
        yaw = robomaster.util.GIMBAL_YAW_MOVE_CHECKER.val2proto(yaw)
    else:
        pitch = robomaster.util.GIMBAL_PITCH_TARGET_CHECKER.val2proto(pitch)
        yaw = robomaster.util.GIMBAL_YAW_TARGET_CHECKER.val2proto(yaw)
    action = robomaster.gimbal.GimbalMoveAction(pitch, yaw, pitch_speed, yaw_speed, frame)
    api._action_dispatcher.send_action(action)
    return action


class Gimbal(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.gimbal
        self.api.recenter()
        self.robot = robot
        self.clock = node.get_clock()
        self.logger = node.get_logger()
        self.node = node
        mode: int = node.declare_parameter('mode', 2).value
        self._mode = mode2api(mode)
        robot.set_robot_mode(self._mode)
        self.logger.info(f"Set initial robot mode to {self._mode}")
        self.joint_msg = sensor_msgs.msg.JointState()
        self.joint_msg.name = [self.node.tf_frame(name) for name
                               in ('gimbal_joint', 'blaster_joint')]

        gimbal_rate = rate(node, 'gimbal', 10)
        if gimbal_rate:
            self.subscribe(gimbal_rate)
        node.create_subscription(robomaster_msgs.msg.GimbalCommand, 'cmd_gimbal',
                                 self.has_received_command, 1)
        # Latched
        node.create_subscription(robomaster_msgs.msg.Mode, 'gimbal/mode', self.has_received_mode, 1)
        node.create_subscription(std_msgs.msg.Bool, 'gimbal/lock', self.has_received_lock, 1)
        node.create_subscription(std_msgs.msg.Bool, 'gimbal/engage', self.has_received_engage, 1)
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # node.create_service(std_srvs.srv.SetBool, 'gimbal/engage', self.set_status_cb)
        self._move_gimbal_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveGimbal, 'move_gimbal',
            self.execute_move_gimbal_callback,
            goal_callback=self.new_gimbal_goal_callback,
            cancel_callback=self.cancel_gimbal_callback,
            callback_group=cbg)
        # Do I need the recenter action too? ... let's add it ... then I decide if I should keep it
        self._recenter_gimbal_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.RecenterGimbal, 'recenter_gimbal',
            self.execute_recenter_gimbal_callback,
            goal_callback=self.new_gimbal_goal_callback,
            cancel_callback=self.cancel_gimbal_callback,
            callback_group=cbg)
        self.action: Optional[robomaster.action.Action] = None
        node.add_on_set_parameters_callback(self.set_params_cb)

    def subscribe(self, rate: Rate) -> None:
        if rate:
            self.api.sub_angle(freq=rate, callback=self.updated_gimbal_position)

    def set_params_cb(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        for param in params:
            if 'gimbal' not in param.name:
                continue
            if param.name == 'gimbal.rate':
                param.value = nearest_rate(param.value)
                self.subscribe(param.value)
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def stop(self) -> None:
        if self.node.connected:
            self.api.unsub_angle()
        self._move_gimbal_action_server.destroy()
        self._recenter_gimbal_action_server.destroy()

    def abort(self) -> None:
        if self.action:
            self.action._abort()
            while self.action is not None:
                self.logger.info("wait for the action to terminate")
                time.sleep(0.1)

    def engage(self, value: bool) -> None:
        if value:
            self.api.resume()
        else:
            self.api.suspend()

    def set_engage_cb(self, request: std_srvs.srv.SetBool.Request,
                      response: std_srvs.srv.SetBool.Response) -> std_srvs.srv.SetBool.Response:
        self.engage(request.data)
        response.success = True
        return response

    def has_received_mode(self, msg: robomaster_msgs.msg.Mode) -> None:
        self._mode = mode2api(msg.mode)
        self.robot.set_robot_mode(self._mode)

    def has_received_engage(self, msg: std_msgs.msg.Bool) -> None:
        self.engage(msg.data)

    def has_received_lock(self, msg: std_msgs.msg.Bool) -> None:
        # self.logger.info("has_received_lock")
        if msg.data:
            self.robot.set_robot_mode(robomaster.robot.CHASSIS_LEAD)
            try:
                self.logger.info("Will lock")
                # Sent at least a (unit) 1, else it won't execute the action
                # on board and change state
                # TODO(Jerome): check timeouts
                move(self.api, yaw=0.5).wait_for_completed(timeout=2)
                move(self.api, yaw=-0.5).wait_for_completed(timeout=2)
                self.logger.info("Locked")
            except Exception as e:  # noqa
                self.logger.warning(f'Cannot move gimbal: {e}')

    def has_received_command(self, msg: robomaster_msgs.msg.GimbalCommand) -> None:
        # self.logger.info(f'has_received_command {deg(msg.pitch_speed)} {deg(msg.yaw_speed)}')
        self.api.drive_speed(pitch_speed=-deg(msg.pitch_speed), yaw_speed=deg(msg.yaw_speed))

    # (pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle)
    def updated_gimbal_position(self, msg: GimbalData) -> None:
        self.joint_msg.header.stamp = self.clock.now().to_msg()
        self.joint_msg.position = [rad(-msg[1]), rad(-msg[0])]
        self.node.joint_state_pub.publish(self.joint_msg)

    # TODO: actions are NOT threaded -> they block other callbacks
    # so the hearbeat deadman trigger and the node exit
    # Actions should not block! ... I should keep receiving and forwarding data
    def execute_move_gimbal_callback(self, goal_handle: Any
                                     ) -> robomaster_msgs.action.MoveGimbal.Result:
        if self.action:
            # should never be, as mutually excluive:
            self.logger.warning('Cannot move gimbal: already performing action')
            goal_handle.abort()
            return robomaster_msgs.action.RecenterGimbal.Result()
        request = goal_handle.request
        # DONE(Jerome): exposes all coordinates frames?
        # if request.relative:
        #     f = self.api.move
        # else:
        #     f = self.api.move_to
        try:
            self.action = move(
                self.api, yaw=-deg(request.yaw), pitch=-deg(request.pitch),
                pitch_speed=deg(request.pitch_speed), yaw_speed=deg(request.yaw_speed),
                frame=request.frame)
        except Exception as e:  # noqa
            self.logger.warning(f'Cannot move gimbal: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveGimbal.Result()
        feedback_msg = robomaster_msgs.action.MoveGimbal.Feedback()

        def cb() -> None:
            feedback_msg.progress = cast(robomaster.action.Action, self.action)._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        self.logger.info(f'Start moving gimbal with request {request}')
        wait_action(self.action, cb)
        # TODO(Jerome) add timeout
        # self.move_action.wait_for_completed()
        if self.action.has_succeeded:
            self.logger.info('Done moving gimbal: succeed')
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            self.logger.info('Done moving gimbal: canceled')
            goal_handle.canceled()
        else:
            self.logger.warning('Done moving gimbal: aborted')
            try:
                goal_handle.abort()
            except rclpy._rclpy_pybind11.RCLError:
                # Happens when the context shutdown before we send feedbacks
                pass
        self.action = None

        return robomaster_msgs.action.MoveGimbal.Result()

    def execute_recenter_gimbal_callback(self, goal_handle: Any
                                         ) -> robomaster_msgs.action.RecenterGimbal.Result:
        if self.action:
            # should never be, as mutually excluive:
            self.logger.warning('Cannot move gimbal: already performing action')
            goal_handle.abort()
            return robomaster_msgs.action.RecenterGimbal.Result()
        request = goal_handle.request
        try:
            self.action = self.api.recenter(
                pitch_speed=int(deg(request.pitch_speed)), yaw_speed=int(deg(request.yaw_speed)))
        except Exception as e:  # noqa
            self.logger.warning(f'Cannot move gimbal: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.RecenterGimbal.Result()
        feedback_msg = robomaster_msgs.action.RecenterGimbal.Feedback()

        def cb() -> None:
            feedback_msg.progress = cast(
                robomaster.action.Action, self.action)._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        self.logger.info(f'Start recentering gimbal after request {request}')
        wait_action(self.action, cb)
        # TODO(Jerome) add timeout
        # self.recenter_action.wait_for_completed()
        if self.action.has_succeeded:
            self.logger.info('Done recentering gimbal: succeed')
            goal_handle.succeed()
        elif goal_handle.is_cancel_requested:
            self.logger.info('Done recentering gimbal: canceled')
            goal_handle.canceled()
        else:
            self.logger.warning('Done recentering gimbal: aborted')
            try:
                goal_handle.abort()
            except rclpy._rclpy_pybind11.RCLError:
                # Happens when the context shutdown before we send feedbacks
                pass
        self.action = None
        return robomaster_msgs.action.RecenterGimbal.Result()

    def _stop_action(self) -> None:
        if self.action:
            self.logger.info('Canceling gimbal action')
            # ABORTED, which would be a more corrent state, is not handled as a completed state
            # by the SDK, therefore we use FAILED
            self.action._changeto_state(robomaster.action.ACTION_FAILED)
            # TODO: tentative
            move(self.api, yaw=0, pitch=0).wait_for_completed(timeout=2)

    def new_gimbal_goal_callback(self, goal_request: Any) -> rclpy.action.server.GoalResponse:
        if self.action:
            return rclpy.action.server.GoalResponse.REJECT
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_gimbal_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        # self.logger.warn('It is not possible to cancel onboard actions')
        # return rclpy.action.CancelResponse.REJECT
        if self.action:
            self.node.executor.create_task(self._stop_action)
        return rclpy.action.CancelResponse.ACCEPT
