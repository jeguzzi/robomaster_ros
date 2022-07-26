.. ros:currentnode:: RoboMasterROS

==============
Sensor adapter
==============

.. warning:: Documentation to be completed

Parameter
---------

.. ros:parameter:: sensor_adapter.enabled bool
  :default: false

.. ros:parameter:: sensor_adapter.rate int
  :default: 10


Publishers
----------

.. ros:publisher:: sensor_adapter robomaster_msgs/SensorAdapter


Service Servers
---------------

.. ros:service_server:: get_adc robomaster_msgs/GetADC
  :qos-reliability: reliable
  :qos-durability: volatile

.. ros:service_server:: get_io robomaster_msgs/GetIO
  :qos-reliability: reliable
  :qos-durability: volatile

.. ros:service_server:: get_pulse robomaster_msgs/GetPulse
  :qos-reliability: reliable
  :qos-durability: volatile
