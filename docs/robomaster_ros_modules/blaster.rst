.. ros:currentnode:: RoboMasterROS

=======
Blaster
=======

This module wraps the SDK class :py:class:`rm:robomaster.blaster.Blaster` to expose a
minimal ROS interface to control the strong green LED of the blaster.

When the module enabled, at initialization, the LED is switched off.

.. note::
  We have not exposed in ROS the command to fire IR beams and gel beams.

Parameters
----------

.. ros:parameter:: blaster.enabled bool
  :default: false

  Enable :ros:sub:`blaster_led`, which by default is disabled, like for all other modules.


Subscription
------------

.. ros:subscription:: blaster_led robomaster_msgs/BlasterLED

  When it receive a message, it passes the clamp the desired relative LED intensity in (0, 1) and
  passes it to :py:meth:`rm:robomaster.blaster.Blaster.set_led`.
  Desired intensities lower or equal to zero, switch the LED off.
