.. ros:currentnode:: RoboMasterROS

====
UART
====

.. warning:: Documentation to be completed

.. :py:meth:`rm:robomaster.serial.sub_serial_msg`


Parameter
---------

.. ros:parameter:: uart.enabled bool
  :default: false

.. ros:parameter:: uart.rx.enable bool
  :default: true

.. ros:parameter:: uart.tx.enable int
  :default: true

.. ros:parameter:: uart.rx.size int
  :default: 50

.. ros:parameter:: uart.tx.size int
  :default: 50

.. ros:parameter:: uart.baud_rate int
  :default: 115200

.. ros:parameter:: uart.data_bit int
  :default: 0

.. ros:parameter:: uart.odd_even int
  :default: 0

.. ros:parameter:: uart.stop_bit int
  :default: 0


Subscriptions
-------------

.. ros:publisher:: uart/rx robomaster_msgs/Serial

Publishers
----------

.. ros:subscription:: uart/rx robomaster_msgs/Serial
