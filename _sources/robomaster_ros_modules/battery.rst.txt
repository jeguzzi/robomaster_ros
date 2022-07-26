.. ros:currentnode:: RoboMasterROS

=======
Battery
=======

This module republishes the battery state collected at 1 Hz from
:py:meth:`rm:robomaster.battery.Battery.sub_battery_info` to ROS topic :ros:pub:`battery`.
We patched the SDK callback to expose voltage, current, and temperature.

LiPo battery has a nominal capacity of 2.4 Ah (25.92 Wh) and nominal voltage of 10.8 V.

.. note::
  To expose the battery state on the gimbal (when available), run executable
  :ros:exec:`robomaster_ros/display_battery`.

Parameters
----------

.. ros:parameter:: battery.enabled bool
  :default: false

  Enable :ros:pub:`battery`, which by default is disabled, like for all other modules.


Publishers
----------

.. ros:publisher:: battery sensor_msgs/BatteryState
  :qos-reliability: reliable
  :qos-durability: volatile

  Regularly publishes the current battery state. Current charge and capacity are unknown and set to -1.
