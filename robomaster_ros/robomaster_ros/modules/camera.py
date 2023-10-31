import enum
import queue
import threading
from typing import Union

import numpy as np
import rcl_interfaces.msg
import rclpy
import rclpy.duration
import rclpy.time
import robomaster.media
import robomaster_msgs.msg
import sensor_msgs.msg
import yaml
from cv_bridge import CvBridge

try:
    from rclpy.event_handler import (PublisherEventCallbacks,
                                     QoSPublisherMatchedInfo)
    EVENT_HANDLER_AVAILABLE = True
except ImportError:
    EVENT_HANDLER_AVAILABLE = False

try:
    from ffmpeg_image_transport_msgs.msg import FFMPEGPacket

    from .h264_parser import H264Parser
    FFMPEG_AVAILABLE = True
except ImportError:
    FFMPEG_AVAILABLE = False

from typing import TYPE_CHECKING, Any, Dict, Optional

if TYPE_CHECKING:
    from ..client import RoboMasterROS

from .. import Module


class ActiveMode(enum.Enum):
    OFF = 0
    ON = 1
    ON_DEMAND = 2
    DISABLED = -1


# taking 2^14 as maximal amplitude (verified), not 2^15 as could be expected for 16 bit audio
MAX_LEVEL = 10 * 2 * 14 * np.log10(2)


def sound_level(samples: np.ndarray) -> float:
    rms = np.mean(np.power(samples.astype(float), 2))
    return 10 * np.log10(rms) - MAX_LEVEL


def camera_info_from_calibration(calibration: Dict[str, Any],
                                 frame_id: str) -> sensor_msgs.msg.CameraInfo:
    msg = sensor_msgs.msg.CameraInfo()
    msg.header.frame_id = frame_id
    msg.height = calibration['image_height']
    msg.width = calibration['image_width']
    msg.distortion_model = calibration['distortion_model']
    msg.d = calibration['distortion_coefficients']['data']
    msg.k = calibration['camera_matrix']['data']
    msg.r = calibration['rectification_matrix']['data']
    msg.p = calibration['projection_matrix']['data']
    return msg


class Camera(robomaster.media.LiveView, Module):  # type: ignore

    def __init__(self, robot: robomaster.robot.Robot,
                 node: 'RoboMasterROS') -> None:
        self._video_streaming = False
        self.node = node
        robomaster.media.LiveView.__init__(self, robot)
        self.logger = node.get_logger()
        self.api = robot.camera
        self.api._liveview = self
        self.bridge = CvBridge()
        self._message_queue: "queue.Queue[sensor_msgs.msg.Image]" = queue.Queue(
            1)
        self.clock = node.get_clock()
        self.next_capture_time = None
        self.video_is_active = False
        self._video_height = 360
        self._video_raw = ActiveMode.OFF
        self._video_h264 = ActiveMode.OFF
        self._video_ffmpeg = ActiveMode.OFF
        self._video_raw_subscribers = 0
        self._video_h264_subscribers = 0
        self._video_ffmpeg_subscribers = 0

        video_raw = node.declare_parameter("camera.video.raw", 2).value
        video_h264 = node.declare_parameter("camera.video.h264", 2).value
        video_ffmpeg = node.declare_parameter("camera.video.ffmpeg", 2).value

        protocol: str = node.declare_parameter("camera.video.protocol",
                                               "tcp").value
        self.resolution = node.declare_parameter("camera.video.resolution",
                                                 self._video_height).value
        rate: float = node.declare_parameter("camera.video.rate", -1.0).value
        self.min_video_period: Optional[rclpy.duration.Duration]
        if rate > 0:
            self.logger.info(f"[Camera] Fps limited to {rate:.1f}")
            self.min_video_period = rclpy.duration.Duration(nanoseconds=1e9 /
                                                            rate)
        else:
            self.min_video_period = None
        node.create_subscription(robomaster_msgs.msg.CameraConfig,
                                 'camera/config',
                                 self.has_updated_camera_config, 1)
        if protocol in ('udp', 'tcp'):
            robomaster.config.ep_conf.video_stream_proto = protocol
        event_callbacks = None
        if video_raw >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.video_raw_pub_event_cb)
            self.raw_video_pub = node.create_publisher(
                sensor_msgs.msg.Image,
                'camera/image_color',
                1,
                event_callbacks=event_callbacks)
        if video_h264 >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.video_h264_pub_event_cb)
            self.h264_video_pub = node.create_publisher(
                robomaster_msgs.msg.H264Packet,
                'camera/image_h264',
                1,
                event_callbacks=event_callbacks)
        if FFMPEG_AVAILABLE and video_ffmpeg >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.video_ffmpeg_pub_event_cb)
            self.ffmpeg_video_pub = node.create_publisher(
                FFMPEGPacket,
                'camera/image_color/ffmpeg',
                1,
                event_callbacks=event_callbacks)

        self.frame_id = node.tf_frame('camera_optical_link')
        calibration_path = node.declare_parameter(
            "camera.video.calibration_file", '').value
        self.should_publish_camera_info = False

        self.has_video = max(video_raw, video_h264, video_ffmpeg) >= 0

        if calibration_path and self.has_video:
            try:
                with open(calibration_path, 'r') as f:
                    calibration = yaml.load(f, yaml.SafeLoader)
                    if calibration:
                        self.camera_info_msg = camera_info_from_calibration(
                            calibration, self.frame_id)
                        self.camera_info_pub = node.create_publisher(
                            sensor_msgs.msg.CameraInfo, 'camera/camera_info',
                            1)
                        self.should_publish_camera_info = True
            except FileNotFoundError:
                self.logger.warn(
                    f"[Camera] Calibration file not found at {calibration_path}"
                )
        # self.logger.info(f"[Camera] Start video stream with resolution {height}p")
        # self.api.start_video_stream(display=False, resolution=f"{height}p")

        self.video_raw = video_raw
        self.video_h264 = video_h264
        self.video_ffmpeg = video_ffmpeg

        self.audio_is_active = False
        self._audio_raw = ActiveMode.OFF
        self._audio_opus = ActiveMode.OFF
        self._audio_level = ActiveMode.OFF
        self._audio_raw_subscribers = 0
        self._audio_opus_subscribers = 0
        self._audio_level_subscribers = 0

        audio_raw = node.declare_parameter("camera.audio.raw", 2).value
        audio_opus = node.declare_parameter("camera.audio.opus", 2).value
        audio_level = node.declare_parameter("camera.audio.level",2).value

        event_callbacks = None
        if audio_raw >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.audio_raw_pub_event_cb)
            self.audio_raw_pub = node.create_publisher(
                robomaster_msgs.msg.AudioData,
                'camera/audio_raw',
                1,
                event_callbacks=event_callbacks)
        if audio_opus >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.audio_opus_pub_event_cb)
            self.audio_opus_pub = node.create_publisher(
                robomaster_msgs.msg.AudioOpus,
                'camera/audio_opus',
                1,
                event_callbacks=event_callbacks)
        if audio_level >= 0:
            if EVENT_HANDLER_AVAILABLE:
                event_callbacks = PublisherEventCallbacks(
                    matched=self.audio_level_pub_event_cb)
            self.audio_level_pub = node.create_publisher(
                robomaster_msgs.msg.AudioLevel,
                'camera/audio_level',
                1,
                event_callbacks=event_callbacks)
        # self.api.start_audio_stream()

        count_subs_period_s = node.declare_parameter(
            "camera.count_subs_period", 1.0).value

        self.audio_raw = audio_raw
        self.audio_opus = audio_opus
        self.audio_level = audio_level

        self.has_audio = max(audio_raw, audio_opus, audio_level) >= 0

        if not EVENT_HANDLER_AVAILABLE and (self.has_audio or self.has_video):
            self.count_subs_timer = node.create_timer(count_subs_period_s,
                                                      self.count_subs_cb)
        else:
            self.count_subs_timer = None

        # TODO(Jerome): disabled because setting the resolution is not working ... why?
        node.add_on_set_parameters_callback(self.set_params_cb)

    def count_subs_cb(self) -> None:
        if self.video_h264 != ActiveMode.DISABLED:
            self._video_h264_subscribers = self.node.count_subscribers(
                'camera/image_h264')
        if self.video_raw != ActiveMode.DISABLED:
            self._video_raw_subscribers = self.node.count_subscribers(
                'camera/image_color')
        if self.video_ffmpeg != ActiveMode.DISABLED:
            self._video_ffmpeg_subscribers = self.node.count_subscribers(
                'camera/image_color/ffmpeg')
        if self.audio_raw != ActiveMode.DISABLED:
            self._audio_raw_subscribers = self.node.count_subscribers(
                'camera/audio_raw')
        if self.audio_opus != ActiveMode.DISABLED:
            self._audio_opus_subscribers = self.node.count_subscribers(
                'camera/audio_opus')
        if self.audio_level != ActiveMode.DISABLED:
            self._audio_level_subscribers = self.node.count_subscribers(
                'camera/audio_level')
        if self.has_video:
            self.check_video()
        if self.has_audio:
            self.check_audio()

    @property
    def resolution(self) -> int:
        return self._video_height

    @resolution.setter
    def resolution(self, value: int) -> None:
        if value != self._video_height:
            if value not in (360, 540, 720):
                self.logger.warning(
                    "[Camera] Resolution {value} not supported: should be 360, 540, or 720"
                )
                return
            self._video_height = value
            if self.video_is_active:
                self.stop_video()
                self.start_video()

    def set_params_cb(self,
                      params: Any) -> rcl_interfaces.msg.SetParametersResult:
        success = True
        for param in params:
            if param.name == 'camera.video.resolution':
                if param.value in (360, 540, 720):
                    self.logger.info(
                        f"[Camera] Set resolution to {param.value}p")
                    self.resolution = param.value
                else:
                    success = False
            if param.name == 'camera.video.raw' and self.video_raw != ActiveMode.DISABLED:
                self.video_raw = param.value
            if param.name == 'camera.video.h264' and self.video_h264 != ActiveMode.DISABLED:
                self.video_h264 = param.value
            if param.name == 'camera.video.ffmpeg' and self.video_ffmpeg != ActiveMode.DISABLED:
                self.video_ffmpeg = param.value
            if param.name == 'camera.audio.raw' and self.audio_raw != ActiveMode.DISABLED:
                self.audio_raw = param.value
            if param.name == 'camera.audio.opus' and self.audio_opus != ActiveMode.DISABLED:
                self.audio_opus = param.value
            if param.name == 'camera.audio.level' and self.audio_level != ActiveMode.DISABLED:
                self.audio_level = param.value
        return rcl_interfaces.msg.SetParametersResult(successful=success)

    def video_raw_pub_event_cb(self, info: 'QoSPublisherMatchedInfo') -> None:
        self._video_raw_subscribers = info.current_count
        self.check_video()

    def video_h264_pub_event_cb(self, info: 'QoSPublisherMatchedInfo') -> None:
        self._video_h264_subscribers = info.current_count
        self.check_video()

    def video_ffmpeg_pub_event_cb(self,
                                  info: 'QoSPublisherMatchedInfo') -> None:
        self._video_ffmpeg_subscribers = info.current_count
        self.check_video()

    def audio_raw_pub_event_cb(self, info: 'QoSPublisherMatchedInfo') -> None:
        self._audio_raw_subscribers = info.current_count
        self.check_audio()

    def audio_opus_pub_event_cb(self, info: 'QoSPublisherMatchedInfo') -> None:
        self._audio_opus_subscribers = info.current_count
        self.check_audio()

    def audio_level_pub_event_cb(self,
                                 info: 'QoSPublisherMatchedInfo') -> None:
        self._audio_level_subscribers = info.current_count
        self.check_audio()

    @property
    def video_should_be_active(self) -> bool:
        return self.should_publish_video_raw or self.should_publish_video_h264 or self.should_publish_video_ffmpeg

    @property
    def should_publish_video_raw(self) -> bool:
        return self.video_raw == ActiveMode.ON or (
            self.video_raw == ActiveMode.ON_DEMAND
            and self._video_raw_subscribers > 0)

    @property
    def should_publish_video_h264(self) -> bool:
        return self.video_h264 == ActiveMode.ON or (
            self.video_h264 == ActiveMode.ON_DEMAND
            and self._video_h264_subscribers > 0)

    @property
    def should_publish_video_ffmpeg(self) -> bool:
        if not FFMPEG_AVAILABLE:
            return False
        return self.video_ffmpeg == ActiveMode.ON or (
            self.video_ffmpeg == ActiveMode.ON_DEMAND
            and self._video_ffmpeg_subscribers > 0)

    def check_video(self) -> None:
        desired = self.video_should_be_active
        if self.video_is_active != desired:
            if desired:
                self.start_video()
            else:
                self.stop_video()

    @property
    def video_raw(self) -> ActiveMode:
        return self._video_raw

    @video_raw.setter
    def video_raw(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if self._video_raw != value:
            self._video_raw = value
            self.next_capture_time = None
            self.check_video()

    @property
    def video_h264(self) -> ActiveMode:
        return self._video_h264

    @video_h264.setter
    def video_h264(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if self._video_h264 != value:
            self._video_h264 = value
            self.check_video()

    @property
    def video_ffmpeg(self) -> ActiveMode:
        return self._video_ffmpeg

    @video_ffmpeg.setter
    def video_ffmpeg(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if value != ActiveMode.OFF and not FFMPEG_AVAILABLE:
            self.logger.warn(
                "[Camera] FFmpeg not available: install PyAV and ffmpeg_image_transport_msgs"
            )
            return
        if self._video_ffmpeg != value:
            self._video_ffmpeg = value
            self.check_video()

    def start_video(self) -> None:
        if not self.video_is_active:
            if self.api.start_video_stream(display=False,
                                           resolution=f"{self.resolution}p"):
                self.video_is_active = True
                self.logger.info(
                    f"[Camera] Started video stream with resolution {self.resolution}p"
                )
            else:
                self.logger.info(f"[Camera] Failed starting video stream")

    def stop_video(self) -> None:
        if self.video_is_active:
            self.api.stop_video_stream()
            self.video_is_active = False
            self.logger.info("[Camera] Stopped video stream")

    def start_video_stream(self,
                           display: bool = False,
                           addr: Optional[str] = None,
                           ip_proto: str = "tcp") -> bool:
        r = super().start_video_stream(display=display,
                                       addr=addr,
                                       ip_proto=ip_proto)
        if r:
            self._video_publisher_thread = threading.Thread(
                target=self._video_publisher_task)
            self._video_publisher_thread.start()
        return r

    def stop_video_stream(self) -> bool:
        if not self.video_is_active:
            return True
        # self.logger.info("will stop_video_stream")
        r = super().stop_video_stream()
        if r:
            self._message_queue.queue.clear()
            self._video_publisher_thread.join()
        return r

    # TODO(jerome) Check because we are overwriting a method here!
    # def stop(self) -> None:
    #     self.logger.info("[Camera] Will stop")
    #     if self.video:
    #         self.stop_video_stream()
    #         self.api.stop_video_stream()
    #     if self.audio:
    #         self.stop_audio_stream()
    #         self.api.stop_audio_stream()
    #     self.logger.info("[Camera] Has stopped")

    # TODO(jerome): maybe better a parameter??
    def has_updated_camera_config(
            self, msg: robomaster_msgs.msg.CameraConfig) -> None:
        self.api._set_zoom(msg.zoom)

    # DONE(jerome): make it work even if the publisher (and/or decoder are slow)
    # TODO(Jerome): why not switching back to the super class method?
    def _video_decoder_task(self) -> None:
        self._video_streaming = True
        seq = 0
        if FFMPEG_AVAILABLE:
            parser = H264Parser()
        while self._video_streaming:
            # blocking
            data = self._video_stream_conn._sock_queue.get()
            while True:
                # empty the queue
                try:
                    additional_data = self._video_stream_conn._sock_queue.get(
                        False)
                    if additional_data:
                        data += additional_data
                except queue.Empty:
                    break
            if not data:
                continue
            capture_time = self.clock.now()
            if self.should_publish_video_ffmpeg:
                for is_keyframe, pts, pkt_data in parser.parse(data):
                    msg = FFMPEGPacket()
                    msg.is_bigendian = False
                    msg.flags = 1 if is_keyframe else 0
                    msg.data = pkt_data
                    msg.header.stamp = capture_time.to_msg()
                    msg.header.frame_id = self.frame_id
                    msg.width = self._video_height * 16 // 9
                    msg.height = self._video_height
                    msg.encoding = "libx264"
                    msg.pts = pts
                    self.ffmpeg_video_pub.publish(msg)
            if self.should_publish_video_h264:
                msg = robomaster_msgs.msg.H264Packet()
                msg.header.stamp = capture_time.to_msg()
                msg.header.frame_id = self.frame_id
                msg.data = data
                msg.seq = seq
                seq += 1
                self.h264_video_pub.publish(msg)
            if self.should_publish_video_raw:
                if self.next_capture_time is None:
                    self.next_capture_time = capture_time
                frames = self._h264_decode(data)
                if not frames:
                    continue
                self._video_frame_count += len(frames)
                # self.logger.info(f"Got frames {len(frames)}")
                if self.min_video_period:
                    if capture_time < self.next_capture_time:
                        continue
                for frame in frames[-1:]:
                    # self.logger.info(f"SHAPE {frame.shape}")
                    i_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    # + rclpy.duration.Duration(nanoseconds=3e8)
                    i_msg.header.stamp = capture_time.to_msg()
                    i_msg.header.frame_id = self.frame_id
                    try:
                        self._message_queue.put(i_msg, block=False)
                    except queue.Full:
                        pass
                    else:
                        if self.min_video_period:
                            self.next_capture_time += self.min_video_period

    def _video_publisher_task(self) -> None:
        while self._video_streaming:
            try:
                msg = self._message_queue.get(timeout=1)
            except queue.Empty:
                continue
            # self.logger.info("Got msg")
            try:
                self.raw_video_pub.publish(msg)
            except rclpy._rclpy_pybind11.RCLError:  # type: ignore
                return
            if self.should_publish_camera_info:
                self.camera_info_msg.header.stamp = msg.header.stamp
                self.camera_info_pub.publish(self.camera_info_msg)
            # self.logger.info("Sent msg")

    @property
    def audio_should_be_active(self) -> bool:
        return self.should_publish_audio_raw or self.should_publish_audio_opus or self.should_publish_audio_level

    @property
    def should_publish_audio_raw(self) -> bool:
        return self.audio_raw == ActiveMode.ON or (
            self.audio_raw == ActiveMode.ON_DEMAND
            and self._audio_raw_subscribers > 0)

    @property
    def should_publish_audio_opus(self) -> bool:
        return self.audio_opus == ActiveMode.ON or (
            self.audio_opus == ActiveMode.ON_DEMAND
            and self._audio_opus_subscribers > 0)

    @property
    def should_publish_audio_level(self) -> bool:
        return self.audio_level == ActiveMode.ON or (
            self.audio_level == ActiveMode.ON_DEMAND
            and self._audio_level_subscribers > 0)

    def check_audio(self) -> None:
        desired = self.audio_should_be_active
        if self.audio_is_active != desired:
            if desired:
                self.start_audio()
            else:
                self.stop_audio()

    @property
    def audio_raw(self) -> ActiveMode:
        return self._audio_raw

    @audio_raw.setter
    def audio_raw(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if self._audio_raw != value:
            self._audio_raw = value
            self.check_audio()

    @property
    def audio_opus(self) -> ActiveMode:
        return self._audio_opus

    @audio_opus.setter
    def audio_opus(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if self._audio_opus != value:
            self._audio_opus = value
            self.check_audio()

    @property
    def audio_level(self) -> ActiveMode:
        return self._audio_level

    @audio_level.setter
    def audio_level(self, value: Union[int, ActiveMode]) -> None:
        try:
            value = ActiveMode(value)
        except ValueError:
            return
        if self._audio_level != value:
            self._audio_level = value
            self.check_audio()

    def start_audio(self) -> None:
        if not self.audio_is_active:
            if self.api.start_audio_stream():
                self.audio_is_active = True
                self.logger.info("[Camera] Started audio stream")
            else:
                self.logger.info(f"[Camera] Failed starting audio stream")

    def stop_audio(self) -> None:
        if self.audio_is_active:
            self.api.stop_audio_stream()
            self.audio_is_active = False
            self.logger.info("[Camera] Stopped audio stream")

    def start_audio_stream(self,
                           addr: Optional[str] = None,
                           ip_proto: str = "tcp") -> bool:
        r = super().start_audio_stream(addr=addr, ip_proto=ip_proto)
        if r:
            self._audio_publisher_thread = threading.Thread(
                target=self._audio_publisher_task)
            self._audio_publisher_thread.start()
        return r

    def stop_audio_stream(self) -> bool:
        r = super().stop_audio_stream()
        if r:
            self._audio_publisher_thread = threading.Thread(
                target=self._audio_publisher_task)
            self._audio_publisher_thread.start()
        return r

    def _audio_decoder_task(self) -> None:
        self._audio_streaming = True
        seq = 0
        while self._audio_streaming:
            data = self._audio_stream_conn.read_buf()
            if not data:
                continue
            if self.publish_opus_audio:
                msg = robomaster_msgs.msg.AudioOpus(buffer=data)
                msg.header.stamp = self.clock.now().to_msg()
                msg.seq = seq
                seq += 1
                self.audio_opus_pub.publish(msg)
            frame = self._audio_decoder.decode(data)
            if frame:
                try:
                    self._audio_frame_count += 1
                    self._audio_frame_queue.put(frame, block=False)
                except queue.Full:
                    pass

    def _audio_publisher_task(self) -> None:
        while self._audio_streaming:
            try:
                frame = self._audio_frame_queue.get(timeout=1)
            except queue.Empty:
                continue
            frame = np.frombuffer(frame, np.int16)
            if self.publish_raw_audio:
                msg = robomaster_msgs.msg.AudioData(data=frame)
                msg.header.stamp = self.clock.now().to_msg()
                self.audio_raw_pub.publish(msg)
            if self.should_publish_audio_level:
                level_msg = robomaster_msgs.msg.AudioLevel(
                    level=sound_level(frame))
                self.audio_level_pub.publish(level_msg)
