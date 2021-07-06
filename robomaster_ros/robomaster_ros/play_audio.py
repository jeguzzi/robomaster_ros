from typing import Any

import pyaudio

import rclpy
import rclpy.node
import robomaster_msgs.msg


class Play(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("play")
        self.create_subscription(
            robomaster_msgs.msg.AudioData, 'camera/audio_raw', self.has_received_data, 10)
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,         # use ffprobe to get this from the file beforehand
            rate=48000,         # use ffprobe to get this from the file beforehand
            output=True)

    def has_received_data(self, msg: robomaster_msgs.msg.AudioData) -> None:
        self.stream.write(msg.data.tobytes())

    def __del__(self) -> None:
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Play()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
