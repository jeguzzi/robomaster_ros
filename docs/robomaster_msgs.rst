===============
robomaster_msgs
===============

.. ros:package:: robomaster_msgs
  :summary:

  This package implements all Robomaster specific interfaces used by the driver
  node :ros:node:`RoboMasterROS`.
  Most interfaces maps the arguments of SDK methods (e.g., :ros:action:`Move` maps to arguments
  of :py:meth:`rm:robomaster.chassis.Chassis.move` or the information provided by SDK subscribers
  (e.g., :ros:msg:`ChassisStatus` maps to the callback arguments
  in :py:meth:`rm:robomaster.chassis.Chassis.sub_status`).


Interfaces
----------


.. ros:autointerfaces:: robomaster_msgs
  :messages: Messages
  :services: Services
  :actions: Actions
  :title_level: ^
