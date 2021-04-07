import threading
import queue
import time

from cv_bridge import CvBridge

import robomaster.media
import robomaster_msgs.msg
import sensor_msgs.msg
from typing import TYPE_CHECKING, Optional
if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class Camera(robomaster.media.LiveView, Module):  # type: ignore

    def __init__(self, robot: robomaster.robot.Robot, node: 'RoboMasterROS') -> None:
        self._video_streaming = False
        robomaster.media.LiveView.__init__(self, robot)
        self.bridge = CvBridge()
        self.api = robot.camera
        self._message_queue = queue.Queue(1)
        self.clock = node.get_clock()
        protocol: str = node.declare_parameter("camera.protocol", "tcp").value
        height: int = node.declare_parameter("camera.resolution", 360).value
        self.logger = node.get_logger()
        node.create_subscription(
            robomaster_msgs.msg.CameraConfig, 'camera/config', self.has_updated_camera_config, 1)
        if protocol in ('udp', 'tcp'):
            robomaster.config.ep_conf.video_stream_proto = protocol
        if height not in (360, 540, 720):
            node.get_logger().error("Resolution not supported. Should be 360, 540, or 720")
            return
        self.pub = node.create_publisher(sensor_msgs.msg.Image, 'camera/image', 1)
        self.api._liveview = self
        self.api.start_video_stream(display=False, resolution=f"{height}p")

    def start_video_stream(self, display: bool = False, addr: Optional[str] = None,
                           ip_proto: str = "tcp") -> None:
        super().start_video_stream(display=display, addr=addr, ip_proto=ip_proto)
        self._publisher_thread = threading.Thread(target=self._publisher_task)
        self._publisher_thread.start()

    def stop_video_stream(self) -> None:
        super().stop_video_stream()
        self._video_decoder_thread.join()
        self._message_queue.queue.clear()

    # TODO(jerome) Check because we are overwriting a method here!
    def stop(self) -> None:
        self.api.stop_video_stream()

    # TODO(jerome): maybe better a parameter??
    def has_updated_camera_config(self, msg: robomaster_msgs.msg.CameraConfig) -> None:
        self.api._set_zoom(msg.zoom)

    # DONE(jerome): make it work even if the publisher (and/or decoder are slow)
    def _video_decoder_task(self) -> None:
        self._video_streaming = True
        while self._video_streaming:
            # blocking
            data = self._video_stream_conn._sock_queue.get()
            while True:
                # empty the queue
                try:
                    data += self._video_stream_conn._sock_queue.get(False)
                except queue.Empty:
                    break
            if data:
                frames = self._h264_decode(data)
                self._video_frame_count += len(frames)
                for frame in frames[-1:]:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = self.clock.now().to_msg()
                    try:
                        self._message_queue.put(msg, block=False)
                    except queue.Full:
                        pass

    def _publisher_task(self) -> None:
        while self._video_streaming:
            msg = self._message_queue.get()
            self.pub.publish(msg)
