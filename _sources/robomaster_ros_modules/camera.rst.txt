.. ros:currentnode:: RoboMasterROS

Camera
======

This module wraps the SDK class :py:class:`rm:robomaster.camera.Camera` to expose audio and video
streams captured from the camera.
The SDK receives separate streams for encoded video and audio via a dedicated IP socket:
audio using `OPUS <https://opus-codec.org/>`_, video using
`H.264 <https://en.wikipedia.org/wiki/Advanced_Video_Coding>`_.

We patched the image and audio decoding
loops in :py:class:`rm:robomaster.media.LiveView` to publish to ROS,
when the parameters `camera.{audio|video}.{raw|<codec>}` are enabled:

- original encoded audio :ros:pub:`camera/audio_opus` (OPUS) and video :ros:pub:`camera/image_h264`
- decoded audio :ros:pub:`camera/audio_raw` and video :ros:pub:`camera/image_color`

Each time a decoded image is published, we also publish the related camera information
on :ros:pub:`camera/camera_info`.

The camera resolution can be selected (with parameter :ros:param:`camera.video.resolution`) between:
640 x 360, 960 x 540 and 1280 x 720. Frame rate is 30 fps. Digital zoom can be selected
with :ros:sub:`camera/config`.

.. note::

  If you don't need audio or video, it is wisely to disable them.
  In particular, network bandwidth consumption is largely reduced if `camera.video.enabled` is false,
  and CPU consumption may be also largely reduced if `camera.video.raw` is false.

Parameters
----------

.. ros:parameter:: camera.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

.. ros:parameter:: camera.video.enabled bool
  :default: true

  Enables the video stream.

.. ros:parameter:: camera.video.protocol string
  :default: "tcp"

  Select the transport protocol for the video socket: one of ``"tcp"`` and ``"udp"``.

.. ros:parameter:: camera.video.resolution int
  :default: 360

  The height of the 16/9 images in the video stream: one of 360, 540, and 720.

  .. note::

    Larger resolutions requires higher bandwidth and more processing power to decode the frames.

.. ros:parameter:: camera.video.calibration_file string
  :default: ""

  The path to the yaml file with the camera calibration.
  If the path is not empty and the file can be loaded,
  the information is published :ros:pub:`camera/camera_info`.
  Should corresponds to the resolution selected in :ros:param:`camera.video.resolution`

.. ros:parameter:: camera.video.h264 bool
  :default: true

  Enables publishing H.264 packets to :ros:pub:`camera/image_h264`.

.. ros:parameter:: camera.video.raw bool
  :default: true

  Enables decoding frames and publishing them to :ros:pub:`camera/image_color`.

.. ros:parameter:: camera.audio.enabled bool
  :default: true

  Enables the audio stream.

.. ros:parameter:: camera.audio.opus bool
  :default: true

  Enables publishing H.264 packets to :ros:pub:`camera/image_h264`.

.. ros:parameter:: camera.audio.raw bool
  :default: true

  Enables decoding audio samples and publishing them to :ros:pub:`camera/audio_raw`.


Subscription
------------

.. ros:subscription:: camera/config robomaster_msgs/CameraConfig
  :qos-reliability: reliable
  :qos-durability: volatile

  Control the camera, which is limited to set digital zoom.
  The subscription is created only if :ros:param:`camera.video.enabled` is true.

Publishers
----------

.. ros:publisher:: camera/camera_info sensor_msgs/CameraInfo
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes camera information required to interpret geometrically the images each time
  an image in published on :ros:pub:`camera/image_color`.
  This publisher is only created if a valid calibration file is provided
  in :ros:param:`camera.video.calibration_file` and :ros:param:`camera.video.enabled` is true.

.. ros:publisher:: camera/image_color sensor_msgs/Image
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes an image each time a decoded frame is available. Maximal rate is 30 fps.
  Actual rate depends on networking and decoding power.
  The publisher is created only if :ros:param:`camera.video.raw` is true.

.. ros:publisher:: camera/image_h264 robomaster_msgs/H264Packet
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a packet from the H.264 video stream. Rate (about 100 Hz) and size (<= 4096) of the packets vary.
  Bandwidth is about 400 KB/s, which is 2-5% of the bandwidth of :ros:pub:`camera/image_color`.
  The publisher is created only if :ros:param:`camera.video.h264` is true.

.. ros:publisher:: camera/audio_raw robomaster_msgs/AudioData
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes an message each time a new portion of decoded 16-bit audio is available.
  Audio is sampled at 48 KHz: each message contains 960 samples and rate is 50 Hz.
  The publisher is created only if :ros:param:`camera.audio.raw` is true.

.. ros:publisher:: camera/audio_opus robomaster_msgs/AudioOpus
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a packet from the OPUS audio stream.
  Rate is 50 Hz, while size of the packets vary slightly around 240.
  Bandwidth is about 14 KB/s, which is 30% of the bandwidth of :ros:pub:`camera/audio_raw`.
  The publisher is created only if :ros:param:`camera.audio.opus` is true.

.. ros:publisher:: camera/audio_level robomaster_msgs/AudioLevel
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes the audio sound level each time audio samples are published on :ros:pub:`camera/audio_raw`.
  The publisher is created only if :ros:param:`camera.audio.enabled` is true.
