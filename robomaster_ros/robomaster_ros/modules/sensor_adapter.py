from typing import Tuple, List, Optional

import robomaster_msgs.msg
import robomaster_msgs.srv
import robomaster.robot
import robomaster.sensor

import rclpy.qos

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module
from ..utils import rate


SensorAdapterData = Tuple[List[int], List[int]]


class SensorAdapter(Module):

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self.api = robot.sensor_adaptor
        self.node = node
        self.clock = node.get_clock()
        self.pub = node.create_publisher(
            robomaster_msgs.msg.SensorAdapter, 'sensor_adapter',
            rclpy.qos.qos_profile_sensor_data)
        _rate = rate(node, 'sensor_adapter', 0)
        if _rate:
            self.api.sub_adapter(freq=_rate, callback=self.updated_reading)
        node.create_service(robomaster_msgs.srv.GetADC, 'get_adc', self.get_adc_cb)
        node.create_service(robomaster_msgs.srv.GetIO, 'get_io', self.get_io_cb)
        node.create_service(robomaster_msgs.srv.GetPulse, 'get_pulse', self.get_pulse_cb)

    def stop(self) -> None:
        self.api.unsub_adapter()

    def abort(self) -> None:
        pass

    def updated_reading(self, msg: SensorAdapterData) -> None:
        ros_msg = robomaster_msgs.msg.SensorAdapter(io=msg[0], adc=msg[1])
        ros_msg.header.stamp = self.clock.now().to_msg()
        self.pub.publish(ros_msg)

    def get_adc_cb(self, request: robomaster_msgs.srv.GetADC.Request,
                   response: robomaster_msgs.srv.GetADC.Response
                   ) -> robomaster_msgs.srv.GetADC.Response:
        value: Optional[int] = self.api.get_adc(id=request.id, port=request.port)
        if value is not None:
            response.value = value
            response.valid = True
        else:
            response.value = 0
            response.valid = False
        return response

    def get_io_cb(self, request: robomaster_msgs.srv.GetIO.Request,
                  response: robomaster_msgs.srv.GetIO.Response
                  ) -> robomaster_msgs.srv.GetIO.Response:
        value: Optional[int] = self.api.get_io(id=request.id, port=request.port)
        if value is not None:
            response.value = value
            response.valid = True
        else:
            response.value = 0
            response.valid = False
        return response

    def get_pulse_cb(self, request: robomaster_msgs.srv.GetPulse.Request,
                     response: robomaster_msgs.srv.GetPulse.Response
                     ) -> robomaster_msgs.srv.GetPulse.Response:
        value: Optional[int] = self.api.get_pulse(id=request.id, port=request.port)
        if value is not None:
            response.time_ms = value
            response.valid = True
        else:
            response.time_ms = 0
            response.valid = False
        return response
