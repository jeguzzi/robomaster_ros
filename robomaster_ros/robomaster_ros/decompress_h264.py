# We can use several tools to decompress a h264:
# - ffmpeg, libav and their Python library pyav
# - the decoder that comes with the robomaster (libmedia)

from typing import Iterator, Any

import numpy as np

import rclpy
import rclpy.node
from cv_bridge import CvBridge
import sensor_msgs.msg
import robomaster_msgs.msg

Image = np.ndarray


class LibMediaDecoder:

    def __init__(self) -> None:
        import libmedia_codec
        self._video_decoder = libmedia_codec.H264Decoder()

    def decode(self, data: bytes) -> Iterator[np.ndarray]:
        # rclpy.logging.get_logger("decoder").info(f"decode {len(data)} {data[:10]}")
        frames = self._video_decoder.decode(data)
        # rclpy.logging.get_logger("decoder").info(f"-> {len(frames)}")
        for frame_data in frames:
            (frame, width, height, _) = frame_data
            if frame:
                frame = np.frombuffer(frame, dtype=np.ubyte)
                yield frame.reshape((height, width, 3))


# class PyAvDecoder:
    # See https://github.com/appie-17/tello_driver/blob/development/src/test_h264_sub.py


class DecoderNode(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("h264_decoder_node")
        # decoder_name = self.declare_parameter("decoder", "libmedia").value
        self.decoder = LibMediaDecoder()
        self.bridge = CvBridge()
        self.create_subscription(
            robomaster_msgs.msg.H264Packet, 'image_h264', self.has_received_data, 10)
        self.pub = self.create_publisher(sensor_msgs.msg.Image, 'image_raw', 10)

    def has_received_data(self, msg: robomaster_msgs.msg.H264Packet) -> None:
        for frame in self.decoder.decode(msg.data.tobytes()):
            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            image_msg.header = msg.header
            self.pub.publish(image_msg)


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = DecoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
