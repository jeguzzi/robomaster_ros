.. ros:currentnode:: RoboMasterROS

=====
Servo
=====

.. warning:: Documentation to be completed

.. :py:mod:`robomater.servo`
.. `sub_servo_info`

Parameters
----------

.. ros:parameter:: servo.config string
  :default: ''

.. ros:parameter:: servo.rate int
  :default: 10

Publishers
----------

.. ros:publisher:: servo_raw_state robomaster_msgs/ServoRawState

Subscription
------------

.. ros:subscription:: cmd_servo robomaster_msgs/ServoCommand


Action Servers
---------------

.. ros:action_server:: move_servo robomaster_msgs/MoveServo
