import rclpy.action
import robomaster.robot

import robomaster_msgs.action
import robomaster.gimbal
import robomaster.robot
import robomaster_msgs.msg
import sensor_msgs.msg
import std_srvs.srv


from typing import Tuple, Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import add_cb
from ..utils import rate, rad, deg

GimbalData = Tuple[float, float, float, float]


def mode2api(mode: int) -> str:
    if mode == 0:
        return robomaster.robot.FREE
    if mode == 1:
        return robomaster.robot.GIMBAL_LEAD
    return robomaster.robot.CHASSIS_LEAD


def move(api: robomaster.gimbal.Gimbal, pitch: float = 0, yaw: float = 0, pitch_speed: float = 30,
         yaw_speed: float = 0, frame: int = robomaster.gimbal.COORDINATE_CUR) -> robomaster.gimbal.GimbalMoveAction:
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
        robot.set_robot_mode(mode2api(mode))
        self.joint_msg = sensor_msgs.msg.JointState()
        self.joint_msg.name = [self.node.tf_frame(name) for name in ('gimbal_joint', 'blaster_joint')]

        gimbal_rate = rate(node, 'gimbal', 10)
        if gimbal_rate:
            self.api.sub_angle(freq=gimbal_rate, callback=self.updated_gimbal_position)
        node.create_subscription(robomaster_msgs.msg.GimbalCommand, 'cmd_gimbal',
                                 self.has_received_command, 1)
        # Latched
        node.create_subscription(robomaster_msgs.msg.Mode, 'mode',
                                 self.has_received_mode, 1)
        node.create_service(std_srvs.srv.SetBool, 'set_status', self.set_status_cb)
        # Do I need the recenter action too?
        self._move_gimbal_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveGimbal, 'move_gimbal', self.execute_move_gimbal_callback)

    def stop(self) -> None:
        self.api.unsub_angle()
        self._move_gimbal_action_server.destroy()

    def set_status_cb(self, request: std_srvs.srv.SetBool.Request,
                      response: std_srvs.srv.SetBool.Response) -> std_srvs.srv.SetBool.Response:
        if request.data:
            self.api.resume()
        else:
            self.api.suspend()
        response.success = True
        return response

    def has_received_mode(self, msg: robomaster_msgs.msg.Mode) -> None:
        self.robot.set_robot_mode(mode2api(msg.mode))

    def has_received_command(self, msg: robomaster_msgs.msg.GimbalCommand) -> None:
        self.api.drive_speed(pitch_speed=deg(msg.pitch_speed), yaw_speed=deg(msg.yaw_speed))

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
        # TODO(Jerome): Complete with failures, ...
        request = goal_handle.request
        self.logger.info(f'Start moving gimbal with request {request}')
        # DONE(Jerome): exposes all coordinates frames?
        # if request.relative:
        #     f = self.api.move
        # else:
        #     f = self.api.move_to

        action = move(self.api, yaw=-deg(request.yaw), pitch=-deg(request.pitch),
                      pitch_speed=deg(request.pitch_speed), yaw_speed=deg(request.yaw_speed),
                      frame=request.frame)
        feedback_msg = robomaster_msgs.action.MoveGimbal.Feedback()

        def cb() -> None:
            feedback_msg.progress = action._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(action, cb)
        action.wait_for_completed()
        goal_handle.succeed()
        self.logger.info('Done moving gimbal')
        return robomaster_msgs.action.MoveGimbal.Result()
