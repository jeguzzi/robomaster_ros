from cv_bridge import CvBridge

import robomaster.media
import robomaster_msgs.msg
import sensor_msgs.msg
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class Camera(robomaster.media.LiveView, Module):  # type: ignore

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self._video_streaming = False
        robomaster.media.LiveView.__init__(self, robot)
        self.bridge = CvBridge()
        self.api = robot.camera
        self.clock = node.get_clock()
        protocol: str = node.declare_parameter("camera.protocol", "tcp").value
        height: int = node.declare_parameter("camera.resolution", 360).value
        node.create_subscription(
            robomaster_msgs.msg.CameraConfig, 'camera/config', self.has_updated_camera_config, 1)
        if protocol in ('udp', 'tcp'):
            robomaster.config.ep_conf.video_stream_proto = protocol
        if height not in (360, 540, 720):
            node.get_logger().error("Resolution not supported. Should be 360, 540, or 720")
            return
        self.pub = node.create_publisher(sensor_msgs.msg.Image, 'camera/image', 10)
        self.api._liveview = self
        self.api.start_video_stream(display=False, resolution=f"{height}p")

    def stop(self) -> None:
        self.api.stop_video_stream()

    # TODO(jerome): maybe better a parameter??
    def has_updated_camera_config(self, msg: robomaster_msgs.msg.CameraConfig) -> None:
        self.api._set_zoom(msg.zoom)

    def _video_decoder_task(self) -> None:
        self._video_streaming = True
        while self._video_streaming:
            data = b''
            buf = self._video_stream_conn.read_buf()
            if not self._video_streaming:
                break
            if buf:
                data += buf
                frames = self._h264_decode(data)
                for frame in frames[-1:]:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = self.clock.now().to_msg()
                    self.pub.publish(msg)
                    self._video_frame_count += 1
