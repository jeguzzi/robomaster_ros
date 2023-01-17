from typing import Any
from datetime import datetime as dt


import pyaudio
import wave

import rclpy
import rclpy.node
import robomaster_msgs.msg


class Play(rclpy.node.Node):  # type: ignore

    def __init__(self) -> None:
        super().__init__("play")
        self.create_subscription(
            robomaster_msgs.msg.AudioOpus, 'camera/audio_opus', self.has_received_data, 10)
        self.p = pyaudio.PyAudio()
        self.decoder_lib = self.declare_parameter('decoder', 'libmedia').value
        self.save: bool = self.declare_parameter('save', False).value
        if self.decoder_lib == 'libmedia':
            import libmedia_codec
            self.get_logger().info("Use libmedia")
            self.decoder = libmedia_codec.OpusDecoder()
        else:
            from opuslib.classes import Decoder
            self.get_logger().info("Use opuslib")
            self.decoder = Decoder(48000, 1)
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,         # use ffprobe to get this from the file beforehand
            rate=48000,         # use ffprobe to get this from the file beforehand
            output=True)
        if self.save:
            self.wf = wave.open(f"output_{dt.now()}.wav", 'wb')
            self.wf.setnchannels(1)  # mono
            self.wf.setsampwidth(2)  # 16 bit
            self.wf.setframerate(48000)  # 48 KHz

    def has_received_data(self, msg: robomaster_msgs.msg.AudioOpus) -> None:
        buffer = msg.buffer.tobytes()
        if self.decoder_lib == 'libmedia':
            data = self.decoder.decode(buffer)
        else:
            from opuslib.exceptions import OpusError
            try:
                data = self.decoder.decode(buffer, 960)
            except OpusError:
                return
        self.stream.write(data)
        if self.save:
            self.wf.writeframes(data)

    def __del__(self) -> None:
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        if self.save:
            self.wf.close()


def main(args: Any = None) -> None:
    rclpy.init(args=args)
    node = Play()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
