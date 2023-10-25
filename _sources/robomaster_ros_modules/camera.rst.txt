.. ros:currentnode:: RoboMasterROS

Camera
======

This module wraps the SDK class :py:class:`rm:robomaster.camera.Camera` to expose audio and video
streams captured from the camera.
The SDK receives separate streams for encoded video and audio via a dedicated IP socket:
audio using `OPUS <https://opus-codec.org/>`_, video using
`H.264 <https://en.wikipedia.org/wiki/Advanced_Video_Coding>`_.

We patched the image and audio decoding
loops in :py:class:`rm:robomaster.media.LiveView` to publish to ROS:

- original encoded audio :ros:pub:`camera/audio_opus` (OPUS) and video :ros:pub:`camera/image_h264`,
- original encoded video wrapped using a format compatible with ``image_transport`` :ros:pub:`camera/image_color/ffmpeg` (see `ffmpeg_image_transport <https://github.com/berndpfrommer/ffmpeg_image_transport>`_),
- decoded audio :ros:pub:`camera/audio_raw` and video :ros:pub:`camera/image_color`,
- decoded audio level :ros:pub:`camera/audio_level`.

Each time a decoded image is published, we also publish the related camera information
on :ros:pub:`camera/camera_info`.

The camera resolution can be selected (with parameter :ros:param:`camera.video.resolution`) between:
640 x 360, 960 x 540 and 1280 x 720. Frame rate is 30 fps. Digital zoom can be selected
with :ros:sub:`camera/config`.


.. note::

  To avoid streaming or decoding images or sounds when not needed, we expose the parameters
  ``camera.{audio|video}.<format>`` to control the activation of the different topics. They support the following values:

  - -1: topic is not created; this config cannot be changed at runtime,
  - 0: topic is not active, messages are not published,
  - 1: topic is active, messages are published,
  - 2: topic is active on-demand, messages published only when there is at least one subscriber.

  When no audio/video topic is active, the audio/video stream from the robot to the ROS driver is stopped.
  As soon as one topic becomes active, the stream is resumed.

  If you don't need audio or video, it is wise to avoid activating the topics. In particular for raw images, this way you significantly reduce the required network bandwidth and CPU cycles.


Parameters
----------

.. ros:parameter:: camera.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

.. ros:parameter:: camera.video.raw int
  :default: 0 (before ROS 2 iron), 2 (from ROS 2 iron)
  :dynamic:

  Enables the decoded video stream on :ros:pub:`camera/image_color`: 0 (off), 1 (on), 2 (on-demand).

.. ros:parameter:: camera.video.h264 int
  :default: 0 (before ROS 2 iron), 2 (from ROS 2 iron)
  :dynamic:

  Enables the [original] H264 video stream on :ros:pub:`camera/image_h264`: 0 (off), 1 (on), 2 (on-demand).

.. ros:parameter:: camera.video.ffmpeg int
  :default: 0 (before ROS 2 iron), 2 (from ROS 2 iron)
  :dynamic:

  Enables the [original] H264 video stream on :ros:pub:`camera/image_color/ffmpeg`: 0 (off), 1 (on), 2 (on-demand).

.. ros:parameter:: camera.video.protocol string
  :default: "tcp"

  Select the transport protocol for the video socket: one of ``"tcp"`` and ``"udp"``.

.. ros:parameter:: camera.video.resolution int
  :default: 360
  :dynamic:

  The height of the 16/9 images in the video stream: one of 360, 540, and 720.

  .. note::

    Larger resolutions requires higher bandwidth and more processing power to decode the frames.

.. ros:parameter:: camera.video.calibration_file string
  :default: ""

  The path to the yaml file with the camera calibration.
  If the path is not empty and the file can be loaded,
  the information is published :ros:pub:`camera/camera_info`.
  Should corresponds to the resolution selected in :ros:param:`camera.video.resolution`

.. ros:parameter:: camera.audio.raw int
  :default: true
  :dynamic:

  Enables the publishing the decoded raw audio on :ros:pub:`camera/audio_raw`: 0 (off), 1 (on), 2 (on-demand).

.. ros:parameter:: camera.audio.opus int
  :default: true
  :dynamic:

  Enables the publishing the original encoded audio on :ros:pub:`camera/audio_opus`: 0 (off), 1 (on), 2 (on-demand).

.. ros:parameter:: camera.audio.level int
  :default: true
  :dynamic:

  Enables the publishing audio levels on :ros:pub:`camera/audio_level`: 0 (off), 1 (on), 2 (on-demand).

Subscription
------------

.. ros:subscription:: camera/config robomaster_msgs/CameraConfig
  :qos-reliability: reliable
  :qos-durability: volatile

  Controls the camera, which is limited to set digital zoom.

Publishers
----------

.. ros:publisher:: camera/camera_info sensor_msgs/CameraInfo
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes camera information required to interpret geometrically the images each time
  an image in published on :ros:pub:`camera/image_color`.
  This publisher is only created if a valid calibration file is provided
  in :ros:param:`camera.video.calibration_file`.

.. ros:publisher:: camera/image_color sensor_msgs/Image
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes an image each time a decoded frame is available. Maximal rate is 30 fps.
  Actual rate depends on networking and decoding power.
  The publisher is active only if :ros:param:`camera.video.raw` is on (1) or on-demand (2) with a matching subscriber.

.. ros:publisher:: camera/image_h264 robomaster_msgs/H264Packet
  :qos-reliability: reliable
  :qos-durability: volatile

  Forward H.264 packets from the original video stream. Rate (about 100 Hz) and size (<= 4096) of the packets vary.
  Bandwidth is about 400 KB/s, which is 2-5% of the bandwidth of :ros:pub:`camera/image_color`.
  The publisher is active only if :ros:param:`camera.video.h264` is on (1) or on-demand (2) with a matching subscriber.

.. ros:publisher:: camera/image_color/ffmpeg ffmpeg_image_transport_msgs/FFMPEGPacket
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a H264 packets, potentially grouping together multiple packets of the original video stream. Contrary to :ros:pub:`camera/image_h264`, the packet contains a single frame, therefore it is published at 30 fps.
  It requires approximately the same bandwidth as :ros:pub:`camera/image_h264`, i.e., about 400 KB/s, which is 2-5% of the bandwidth of :ros:pub:`camera/image_color`.
  The publisher is active only if :ros:param:`camera.video.ffmpeg` is on (1) or on-demand (2) with a matching subscriber.
  This topic is meant to be used with ``image_transport`` with the ``ffmpeg`` trasport implemented by `ffmpeg_image_transport <https://github.com/berndpfrommer/ffmpeg_image_transport>`_.

.. ros:publisher:: camera/audio_raw robomaster_msgs/AudioData
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes an message each time a new portion of decoded 16-bit audio is available.
  Audio is sampled at 48 KHz: each message contains 960 samples and rate is 50 Hz.
  The publisher is active only if :ros:param:`camera.audio.raw` is on (1) or on-demand (2) with a matching subscriber.

.. ros:publisher:: camera/audio_opus robomaster_msgs/AudioOpus
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a packet from the OPUS audio stream.
  Rate is 50 Hz, while size of the packets vary slightly around 240.
  Bandwidth is about 14 KB/s, which is 30% of the bandwidth of :ros:pub:`camera/audio_raw`.
  The publisher is active only if :ros:param:`camera.audio.opus` is on (1) or on-demand (2) with a matching subscriber.

.. ros:publisher:: camera/audio_level robomaster_msgs/AudioLevel
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes the audio sound level.
  The publisher is active only if :ros:param:`camera.audio.level` is on (1) or on-demand (2) with a matching subscriber.
