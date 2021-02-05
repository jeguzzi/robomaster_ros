import time
import enum

import numpy as np

import rclpy.action

import robomaster.robot
import robomaster.gripper

import robomaster_msgs.msg
import sensor_msgs.msg

from typing import Any
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class GripperState(enum.Enum):
    PAUSED = 0
    OPEN = 1
    CLOSE = 2


def gripper_data_info(self: robomaster.gripper.GripperSubject) -> int:
    return self._status


robomaster.gripper.GripperSubject.data_info = gripper_data_info


class Gripper(Module):

    _gripper_state: int

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.gripper = robot.gripper
        self.node = node
        self.logger = node.get_logger()
        self.clock = node.get_clock()
        self.msg = sensor_msgs.msg.JointState()
        self.msg.name = (
            ['gripper_m_joint'] +
            [f'{side}_gripper_joint_{i}' for side in ('left', 'right') for i in (1, 2, 4, 5, 6, 7)])
        # print(self.msg.name)
        self.data = np.loadtxt("gripper.csv", delimiter=",").T
        # print(len(self.data), self.data)
        n = self.data.shape[-1]
        # TODO: record the prismatic joint too
        self.data = np.insert(self.data, 0, np.linspace(0.001, -0.023, n), axis=0)
        self._xs = np.linspace(0, 1, n)
        self.gripper_pub = node.create_publisher(robomaster_msgs.msg.GripperState, 'gripper', 1)
        self._gripper_state = -1
        self.query_gripper_state()
        # TODO(jerome): make it latched
        self._gripper_action_server = rclpy.action.ActionServer(
            node, robomaster_msgs.action.GripperControl, 'gripper', self.execute_gripper_callback)

    def stop(self) -> None:
        pass

    # 0 open -> 1 close
    def joint_state(self, value: float) -> sensor_msgs.msg.JointState:
        self.msg.position = [np.interp(value, self._xs, data) for data in self.data]
        return self.msg

    def execute_gripper_callback(self, goal_handle: Any
                                 ) -> robomaster_msgs.action.GripperControl.Result:
        # TODO(jerome): Complete with failures, ...
        request = goal_handle.request
        self.logger.info(
            f'[Action begin] gripper goal... {self.gripper_state} -> {request.target_state}')
        if request.target_state == self.gripper_state:
            goal_handle.succeed()
            return robomaster_msgs.action.GripperControl.Result()

        feedback_msg = robomaster_msgs.action.GripperControl.Feedback()
        proto = robomaster.protocol.ProtoGripperCtrl()
        proto._control = request.target_state
        proto._power = robomaster.util.GRIPPER_POWER_CHECK.val2proto(100 * request.power)
        self.logger.info('Sending ProtoGripperCtrl ...')
        self.gripper._send_sync_proto(proto, robomaster.protocol.host2byte(3, 6))
        self.logger.info('ProtoGripperCtrl received')

        def cb(msg: int) -> None:
            feedback_msg.current_state = msg
            goal_handle.publish_feedback(feedback_msg)
            self.gripper_state = msg

        self.gripper.sub_status(freq=10, callback=cb)
        deadline = self.clock.now() + rclpy.duration.Duration(seconds=5)
        while feedback_msg.current_state != request.target_state:
            # print(f'{feedback_msg.current_state} != {request.target_state}')
            time.sleep(1 / 10)
            if(self.clock.now() > deadline):
                break
        self.gripper.unsub_status()
        if feedback_msg.current_state == request.target_state:
            goal_handle.succeed()
            self.logger.info(f'[Action terminate] gripper state {self.gripper_state} == {request.target_state}')
        else:
            goal_handle.abort()
            self.logger.warn(f'[Action failed] gripper state {self.gripper_state} != {request.target_state}')
        return robomaster_msgs.action.GripperControl.Result()

    @property
    def gripper_state(self) -> int:
        return self._gripper_state

    @gripper_state.setter
    def gripper_state(self, value: int) -> None:
        if value != self._gripper_state:
            self.logger.info(f"Gripper state changed to {GripperState(value)}")
            self._gripper_state = value
            self.gripper_pub.publish(robomaster_msgs.msg.GripperState(state=value))
            if(value == robomaster_msgs.msg.GripperState.OPEN):
                self.node.joint_state_pub.publish(self.gripper.joint_state(0))
            if(value == robomaster_msgs.msg.GripperState.CLOSE):
                self.node.joint_state_pub.publish(self.gripper.joint_state(1))

    def query_gripper_state(self) -> None:
        self._gripper_state = -1

        def cb(msg: int) -> None:
            self.gripper_state = msg
        self.gripper.sub_status(freq=10, callback=cb)
        while self._gripper_state == -1:
            time.sleep(0.01)
        self.gripper.unsub_status()
