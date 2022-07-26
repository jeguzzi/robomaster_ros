.. ros:currentnode:: RoboMasterROS

===
PWM
===

.. warning:: Documentation to be completed

Parameter
---------

.. ros:parameter:: pwm.enabled bool
  :default: false

.. ros:parameter:: pwm.frequencies int[]
  :default: [-1, -1, -1, -1, -1, -1]


Subscriptions
-------------

.. ros:subscription:: cmd_pwm robomaster_msgs/PWM,
  :qos-reliability: reliable
  :qos-durability: volatile
