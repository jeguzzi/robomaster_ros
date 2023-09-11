import time

import rclpy.action
import robomaster.robot
import robomaster.robotic_arm
import robomaster.action

import geometry_msgs.msg
import robomaster_msgs.action
import sensor_msgs.msg
import rcl_interfaces.msg

from .servo import ServoData, RMServo

from typing import Dict, Tuple, Any, Optional, cast
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import wait_action, abort_action, remove_action_with_target
from ..utils import rate, nearest_rate, Rate


def arm_joint_state(left_motor: float, right_motor: float) -> Dict[str, float]:
    # return {
    #     'arm_1_joint': right_motor,
    #     'rod_joint': left_motor,
    #     'triangle_joint': -right_motor,
    #     'arm_2_joint': left_motor - right_motor,
    #     'endpoint_bracket_joint': -left_motor,
    #     'rod_1_joint': left_motor,
    #     'rod_3_joint': right_motor - left_motor,
    #     'rod_2_joint': right_motor
    # }
    # Now that the urdf defines dependent joints, we can skip most of them
    return {
        'arm_1_joint': right_motor,
        'rod_joint': left_motor,
        'rod_3_joint': right_motor - left_motor,
    }


class Arm(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.node = node
        self.robot = robot
        right_motor_zero: int = node.declare_parameter('arm.right_motor.zero', 1024).value
        left_motor_zero: int = node.declare_parameter('arm.left_motor.zero', 1024).value
        right_motor_angle: float = node.declare_parameter('arm.right_motor.angle', -0.274016).value
        left_motor_angle: float = node.declare_parameter('arm.left_motor.angle', 0.073304).value
        right_motor_direction: int = node.declare_parameter('arm.right_motor.direction', -1).value
        left_motor_direction: int = node.declare_parameter('arm.left_motor.direction', 1).value
        right_motor_index: int = node.declare_parameter('arm.right_motor.index', 1).value
        left_motor_index: int = node.declare_parameter('arm.left_motor.index', 0).value
        self.servos = {
            'right_motor': RMServo(
                index=right_motor_index, reference_angle=right_motor_angle,
                reference_value=right_motor_zero, direction=right_motor_direction,
                name=node.tf_frame('arm_1_joint')),
            'left_motor': RMServo(
                index=left_motor_index, reference_angle=left_motor_angle,
                reference_value=left_motor_zero, direction=left_motor_direction,
                name=node.tf_frame("rod_joint"))
        }
        node.get_logger().info(
            f"[Arm] left servo: index {left_motor_index}, direction {left_motor_direction}, "
            f"value {left_motor_zero}, angle {left_motor_angle}")
        node.get_logger().info(
            f"[Arm] right servo: index {right_motor_index}, direction {right_motor_direction}, "
            f"value {right_motor_zero}, angle {right_motor_angle}")
        self.arm_position_msg = geometry_msgs.msg.PointStamped()
        self.arm_position_msg.header.frame_id = node.tf_frame('arm_base_link')
        self.arm_position_msg.point.y = 0.0
        self.arm_position_pub = node.create_publisher(
            geometry_msgs.msg.PointStamped, 'arm_position', 1)
        self.robot = robot
        self.clock = node.get_clock()
        self.logger = node.get_logger()
        self.node = node
        self.api = robot.robotic_arm
        arm_rate = rate(node, 'arm', 10)
        if arm_rate:
            self.subscribe(arm_rate)
            self.servo_raw_state_pub = node.create_publisher(
                robomaster_msgs.msg.ServoRawState, 'servo_raw_state', 1)
        cbg = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        # self.logger.info(f"[Arm] Callback group {cbg}")
        self._move_arm_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveArm, 'move_arm', self.execute_move_arm_callback,
            goal_callback=self.new_move_arm_goal_callback,
            cancel_callback=self.cancel_move_arm_callback, callback_group=cbg)
        # self.goal_handle: Optional[rclpy.action.server.ServerGoalHandle] = None
        self.action: Optional[robomaster.action.Action] = None
        node.create_subscription(geometry_msgs.msg.Vector3, 'cmd_arm',
                                 self.has_received_velocity_command, 10)
        node.create_subscription(geometry_msgs.msg.Point, 'target_arm_position',
                                 self.has_received_position_command, 1)
        node.add_on_set_parameters_callback(self.set_params_cb)

    def has_received_velocity_command(self, msg: geometry_msgs.msg.Vector3) -> None:
        # TODO(Jerome): verify that we need to set a new id each time
        # -> no, it seems id = 1 is fine (0 is not)
        # Testing with the joypad, it seems the behavior is not very robust
        # e.g., it seems that sending a 0, 0 will make the arm fall down
        # maybe it releases the motors?
        # I think that it fixes the position at the begin of the action (id)
        # so it's not really relative to the current position but to the position
        # at the first command (at least with action_id = 1)
        x = int(msg.x * 1000)
        z = int(msg.z * 1000)
        # x = 0
        # y = 0
        # self.logger.info(f"delta ({x}, {z}) from ({self.x}, {self.z})")
        # absolute coords works fine
        action = robomaster.robotic_arm.RoboticArmMoveAction(self.x + x, self.z + z, z=0, mode=1)
        # relative have the above problem, even when I abort the [previous] action
        # action = robomaster.robotic_arm.RoboticArmMoveAction(x, z, z=0, mode=0)
        # it could be that relative as a slightly higher range
        # will sending a command break the current action?
        # action._action_id = 1
        action._action_id = action._get_next_action_id()
        msg = self.robot._action_dispatcher.get_msg_by_action(action)
        #  msg._proto._action_ctrl = 0
        # action._action_id -= 1
        # self.robot.client.send_msg(msg)
        # action._action_id += 1
        # msg._proto._action_ctrl = 1
        self.robot.client.send_msg(msg)

    def has_received_position_command(self, msg: geometry_msgs.msg.Point) -> None:
        # self.node.get_logger().info(f"[Arm] arm position command {msg}")
        x = int(msg.x * 1000)
        z = int(msg.z * 1000)
        action = robomaster.robotic_arm.RoboticArmMoveAction(x, z, z=0, mode=1)
        action._action_id = action._get_next_action_id()
        msg = self.robot._action_dispatcher.get_msg_by_action(action)
        self.robot.client.send_msg(msg)


    def subscribe(self, rate: Rate) -> None:
        if rate:
            self.api.sub_position(freq=rate, callback=self.updated_arm_position)
            self.robot.servo.sub_servo_info(freq=rate, callback=self.updated_arm_servo)

    def set_params_cb(self, params: Any) -> rcl_interfaces.msg.SetParametersResult:
        for param in params:
            if 'arm' not in param.name:
                continue
            if param.name == 'arm.rate':
                param.value = nearest_rate(param.value)
                self.subscribe(param.value)
            for name, servo in self.servos.items():
                if name in param.name:
                    if param.name == f'arm.{name}.zero':
                        servo.reference_value = param.value
                    elif param.name == f'arm.{name}.angle':
                        servo.reference_angle = param.value
                    elif param.name == f'arm.{name}.direction':
                        servo.direction = 1 if param.value > 0 else -1
        return rcl_interfaces.msg.SetParametersResult(successful=True)

    def stop(self) -> None:
        self._move_arm_action_server.destroy()
        if self.node.connected:
            self.api.unsub_position()
            self.robot.servo.unsub_servo_info()

    def abort(self) -> None:
        if self.action:
            self.action._abort()
            while self.action is not None:
                self.logger.info(f"[Arm] wait for action {self.action} to terminate")
                time.sleep(0.1)

    def publish_sensor_raw_state(self, msg: ServoData) -> None:
        # self.logger.info(f'publish_sensor_raw_state servo {msg}')
        omsg = robomaster_msgs.msg.ServoRawState()
        for i in range(4):
            omsg.valid[i] = (msg[0][i] != 0)
            omsg.speed[i] = msg[1][i]
            omsg.value[i] = msg[2][i]
        # self.logger.info(f"publish_sensor_raw_state -> {omsg}")
        self.servo_raw_state_pub.publish(omsg)

    # (valid, speed, angle)
    def updated_arm_servo(self, msg: ServoData) -> None:
        # self.logger.info(f'arm servo {msg}')
        for v in self.servos.values():
            v.valid = (msg[0][v.index] != 0)
            if v.valid:
                # v.speed = msg[1][v.index_] * SERVO2RAD
                v.value = msg[2][v.index]
                # print(v.value, v.bias, v.angle)
        right_motor = self.servos['right_motor']
        left_motor = self.servos['left_motor']
        if (right_motor.valid and left_motor.valid):
            arm_state_msg = sensor_msgs.msg.JointState()
            arm_state_msg.header.stamp = self.clock.now().to_msg()
            joints = arm_joint_state(right_motor=right_motor.angle, left_motor=left_motor.angle)
            arm_state_msg.name = [self.node.tf_frame(name) for name in joints.keys()]
            arm_state_msg.position = list(joints.values())
            self.node.joint_state_pub.publish(arm_state_msg)
        for servo in self.servos.values():
            servo.valid = False
        self.publish_sensor_raw_state(msg)

    # TODO: actions are NOT threaded -> they block other callbacks
    # so the hearbeat deadman trigger and the node exit
    # Actions should not block! ... I should keep receving and forwarding data
    def execute_move_arm_callback(self, goal_handle: Any) -> robomaster_msgs.action.MoveArm.Result:
        # TODO(jerome): Complete with failures, ...
        request = goal_handle.request
        # timeout: Optional[float] = None
        timeout: float = 5.0
        if request.relative:
            f = self.api.move
            delta = max(abs(request.x), abs(request.z))
            if delta == 0:
                # HACK(Jerome): else it moves to an undefined target
                goal_handle.succeed()
                self.logger.info('[Arm] No need to move')
                return robomaster_msgs.action.MoveArm.Result()
            # HACK(Jerome): else the teleop +/- x is not concluding the action
            timeout = max(1.0, delta / 0.02)  # 2cm/s
        else:
            f = self.api.moveto
        while True:
            try:
                new_action = f(x=int(request.x * 1000), y=int(request.z * 1000))
                break
            except RuntimeError as e:
                self.logger.warning(
                    f'[Arm] Failed creating new action {e} => remove current action')
                remove_action_with_target(
                    self.robot, robomaster.robotic_arm.RoboticArmMoveAction._target)
        self.action = new_action
        self.logger.info(f'[Arm] Start moving with request {request}: {new_action}')
        feedback_msg = robomaster_msgs.action.MoveArm.Feedback()

        def cb() -> None:
            # self.logger.info(f'Moving arm ... {action._percent}')
            feedback_msg.progress = cast(robomaster.action.Action, new_action)._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)

        # add_cb(self.action, cb)
        # self.action.wait_for_completed(timeout=timeout)
        wait_action(new_action, cb, timeout=timeout)

        if goal_handle.is_cancel_requested:
            self.logger.info('[Arm] Done moving: canceled')
            goal_handle.canceled()
        elif new_action.has_succeeded:
            self.logger.info('[Arm] Done moving: succeeded')
            goal_handle.succeed()
        else:
            self.logger.warning('[Arm] Done moving: aborted')
            try:
                goal_handle.abort()
            except rclpy._rclpy_pybind11.RCLError:
                # Happens when the context shutdown before we send feedbacks
                pass
        self.action = None
        return robomaster_msgs.action.MoveArm.Result()

    def new_move_arm_goal_callback(self, goal_request: robomaster_msgs.action.MoveArm.Goal
                                   ) -> rclpy.action.server.GoalResponse:
        if self.action:
            return rclpy.action.server.GoalResponse.REJECT
        return rclpy.action.server.GoalResponse.ACCEPT

    def cancel_move_arm_callback(self, goal_handle: Any) -> rclpy.action.CancelResponse:
        # self.logger.warn('It is not possible to cancel onboard actions')
        # return rclpy.action.CancelResponse.REJECT
        if self.action:
            self.node.executor.create_task(self._stop_action)
        return rclpy.action.CancelResponse.ACCEPT

    def _stop_action(self) -> None:
        if self.action:
            self.logger.info('[Arm] Canceling action')
            # ABORTED, which would be a more corrent state, is not handled as a completed state
            # by the SDK, therefore we use FAILED
            # Specific:
            # self.action._changeto_state(robomaster.action.ACTION_FAILED)
            # self.api.move(x=0, y=0).wait_for_completed()
            # Generic:
            abort_action(self.robot, self.action)

    # (x, z) in mm
    def updated_arm_position(self, msg: Tuple[int, int]) -> None:
        self.arm_position_msg.header.stamp = self.clock.now().to_msg()
        ps = self.arm_position_msg.point
        ps.x, ps.z = [v * 0.001 for v in msg]
        self.x, self.z = msg
        self.arm_position_pub.publish(self.arm_position_msg)
