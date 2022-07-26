.. ros:currentnode:: RoboMasterROS

=======
Speaker
=======

This module controls the speaker.
It exposes commands to play system sounds indentified by an ID) and wav files.

.. note::

  Sound ids between 0x107 and 0x12A corresponds to three octaves of piano notes, from
  C1=0x107 to B3=0x12A.


Parameter
---------

.. ros:parameter:: speaker.enabled bool
  :default: false


Subscribers
------------

.. ros:subscription:: cmd_sound robomaster_msgs/SpeakerCommand.
  :qos-reliability: reliable
  :qos-durability: volatile

  The command stops or start playing a system sound or a wav file for a number of times.
  Message field ``control`` select the command:

  - 0: stop playing the sound with id ``sound_id``, if any is playing.
  - 1: start playing the sound with id ``sound_id``, stop playing any other sound.
  - 2: start playing the sound with id ``sound_id``, without stop playing other sounds.

  When ``file`` is not empty, the driver plays it, ignoring ``sound_id``.
  The file is loaded through FTP the first time it is played
  and assigned a ``sound_id`` between 0xE0 and 0xEA for successive uses.

  It exposes a similar interface as :ros:act_server:`play` with the difference that multiple sounds
  can be played at the same time and sounds can be selectively stopped.


Action Servers
--------------

.. ros:action_server:: play robomaster_msgs/PlaySound

  The action wraps :py:meth:`rm:robomaster.robot.Robot.play_sound` to play a system sound or a
  wav file for a number of times.
  When ``file`` is not empty, the driver plays it, ignoring ``sound_id``.
  The file is loaded through FTP the first time it is played
  and assigned a ``sound_id`` between 0xE0 and 0xEA for successive uses.

  Action feedback contains the progress and the repetition count.
  The action terminates once all repetitions have been played.

  The action exposes a similar interface as :ros:sub:`cmd_sound`:
  use the action if you want to be notified when the sound finishes playing.
  Contrary to  :ros:sub:`cmd_sound`, the action play
  does not allow to play multiple sounds at the same time and to start a playing a new sound,
  you need to cancel the action.
