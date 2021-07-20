import rclpy.action
import robomaster.robot

import geometry_msgs.msg
import robomaster_msgs.action
import sensor_msgs.msg

from .servo import ServoData, RMServo

from typing import Dict, Tuple, Any, Optional
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..action import add_cb
from ..utils import rate


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
        right_motor_zero: int = node.declare_parameter('arm.right_motor.zero', 1273).value
        left_motor_zero: int = node.declare_parameter('arm.left_motor.zero', 1242).value
        right_motor_direction: int = node.declare_parameter('arm.right_motor.direction', -1).value
        left_motor_direction: int = node.declare_parameter('arm.left_motor.direction', -1).value
        self.servos = {
            'right_motor': RMServo(
                index=1, reference_angle=-0.274016, reference_value=right_motor_zero,
                direction=right_motor_direction, name=node.tf_frame('arm_1_joint')),
            'left_motor': RMServo(
                index=0, reference_angle=0.073304, reference_value=left_motor_zero,
                direction=left_motor_direction, name=node.tf_frame("rod_joint"))
        }
        node.get_logger().info(f"[Arm] zeros: left {left_motor_zero}, right {right_motor_zero}")
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
            self.api.sub_position(freq=arm_rate, callback=self.updated_arm_position)
        self.robot.servo.sub_servo_info(freq=arm_rate, callback=self.updated_arm_servo)
        self._move_arm_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.MoveArm, 'move_arm', self.execute_move_arm_callback)

    def stop(self) -> None:
        self._move_arm_action_server.destroy()
        if self.node.connected:
            self.api.unsub_position()
            self.robot.servo.unsub_servo_info()

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
        if(right_motor.valid and left_motor.valid):
            arm_state_msg = sensor_msgs.msg.JointState()
            arm_state_msg.header.stamp = self.clock.now().to_msg()
            joints = arm_joint_state(right_motor=right_motor.angle, left_motor=left_motor.angle)
            arm_state_msg.name = [self.node.tf_frame(name) for name in joints.keys()]
            arm_state_msg.position = list(joints.values())
            self.node.joint_state_pub.publish(arm_state_msg)
        for servo in self.servos.values():
            servo.valid = False

    # TODO: actions are NOT threaded -> they block other callbacks
    # so the hearbeat deadman trigger and the node exit
    # Actions should not block! ... I should keep receving and forwarding data
    def execute_move_arm_callback(self, goal_handle: Any) -> robomaster_msgs.action.MoveArm.Result:
        # TODO(jerome): Complete with failures, ...
        request = goal_handle.request
        timeout: Optional[float] = None
        if request.relative:
            f = self.api.move
            delta = max(abs(request.x), abs(request.z))
            if delta == 0:
                # HACK(Jerome): else it moves to an undefined target
                goal_handle.succeed()
                self.logger.info('No need to move arm')
                return robomaster_msgs.action.MoveArm.Result()
            # HACK(Jerome): else the teleop +/- x is not concluding the action
            timeout = max(1.0, delta / 0.02)  # 2cm/s
        else:
            f = self.api.moveto
        try:
            action = f(x=int(request.x * 1000), y=int(request.z * 1000))
        except RuntimeError as e:
            self.logger.warning(f'Cannot move arm: {e}')
            goal_handle.abort()
            return robomaster_msgs.action.MoveArm.Result()
        self.logger.info(f'Start moving arm with request {request}')
        feedback_msg = robomaster_msgs.action.MoveArm.Feedback()

        def cb() -> None:
            # self.logger.info(f'Moving arm ... {action._percent}')
            feedback_msg.progress = action._percent * 0.01
            goal_handle.publish_feedback(feedback_msg)
        add_cb(action, cb)
        action.wait_for_completed(timeout=timeout)
        goal_handle.succeed()
        self.logger.info('Done moving arm')
        return robomaster_msgs.action.MoveArm.Result()

    # (x, z) in mm
    def updated_arm_position(self, msg: Tuple[int, int]) -> None:
        self.arm_position_msg.header.stamp = self.clock.now().to_msg()
        ps = self.arm_position_msg.point
        ps.x, ps.z = [v * 0.001 for v in msg]
        self.arm_position_pub.publish(self.arm_position_msg)
