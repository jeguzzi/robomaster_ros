.. ros:currentnode:: RoboMasterROS

======
Vision
======

.. warning:: Documentation to be completed

.. :py:meth:`robomaster.vision.sub_detect_info`


Parameter
---------

.. ros:parameter:: vision.enabled bool
  :default: false

.. ros:parameter:: vision.targets str[]
  :default: ["marker:red", "robot"]


Publishers
----------

.. ros:publisher:: vision robomaster_msgs/Detection
  :qos-reliability: reliable
  :qos-durability: volatile
