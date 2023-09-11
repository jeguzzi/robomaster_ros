=========
Changelog
=========

Unreleased
==========

Added
-----

- Topic ``camera/image_color/ffmpeg`` and parameter ``camera.video.ffmpeg`` to support ``image_transport``.

Changed
-------

- Support for runtime video/audio activation. Parameters ``camera.video.raw``, ``camera.video.h264``, ``camera.audio.raw``, ``camerra.audio.opus`` are now integer-valued and dynamic. Same for the related launch arguments ``video_raw``, ``video_h264``, ``audio_raw``, ``audio_opus``.
- Parameter ``camera.video.resolution`` is now dynamic.

Removed
-------

- Parameters ``camera.video.enabled``, ``camera.audio.enabled`` are superseded by the more fine-grained parameters related to the specific topics.

[0.1] - 2023-09-11
==================

Initial release.

