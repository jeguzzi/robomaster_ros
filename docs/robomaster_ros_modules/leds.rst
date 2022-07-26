.. ros:currentnode:: RoboMasterROS

====
LEDs
====

This module controls the chassis and gimbal leds.
To expose all functionality for :ros:sub:`leds/effect` (e.g., to configure the effect period)
instead of using the SDK method :py:meth:`rm:robomaster.led.Led.set_led`,
it directly send a `robomaster.protocol.ProtoSetSystemLed` to the robot.

The robot will switch off all LEDs when initializing this module.


Parameter
---------

.. ros:parameter:: leds.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

Subscriptions
-------------

.. ros:subscription:: leds/color std_msgs/ColorRGBA
  :qos-reliability: reliable
  :qos-durability: volatile

  A simplified interface to set *all* LEDs to the same color.

.. ros:subscription:: leds/effect robomaster_msgs/LEDEffect
  :qos-reliability: reliable
  :qos-durability: volatile

  The more detailed interface to control LEDs:
  selection of a group of LEDs, color, effect, and on/off periods.
  Effects PULSE and SCROLLING are for gimbal LEDs.

  When using ON effect (i.e., set a solid color), you can specify
  a submask for the portion of gimbal LEDs to switch on (the other portions are switched off).

..
  TODO: check gimbal LEDs:
    - verify that subleds outside the mask are switched off?
    - verify that pulse works
    - describe scrolling
