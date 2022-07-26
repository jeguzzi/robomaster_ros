.. ros:currentnode:: RoboMasterROS

==============================
Time-of-Flight distance sensor
==============================

.. warning:: Documentation to be completed

..
  py:meth:`rm:robomaster.sensor.sub_distance`

Parameter
---------

.. ros:parameter:: vision.enabled bool
  :default: false

.. ros:parameter:: vision.rate int
  :default: 10

Publishers
----------

.. ros:subscription:: range_<index> sensor_msgs/Range
  :qos-reliability: reliable
  :qos-durability: volatile
