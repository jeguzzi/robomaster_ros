===============
Getting Started
===============

.. ros:currentnode:: RoboMasterROS
.. ros:currentpackage:: robomaster_ros

Switch on your Robomaster and connect your PC through
:doc:`one of three available interfaces <rm:python_sdk/connection>`:

USB
  connect using a USB cable to the intelligent controller (:ros:param:`conn_type` = ``"rndis"``)

Wi-Fi Access point
  move the slider on the intelligent controller to the back (:ros:param:`conn_type` = ``"ap"``)
  to make it starts its own access point.
  Connect your PC to it using the SSID and password printed on the top of the intelligent controller.

Wi-Fi Router
  move the slider on the intelligent controller to the front (:ros:param:`conn_type` = ``"sta"``)
  and connect your PC to a Wi-Fi router. To communicate the network SSID and password to the robot,
  you can use the Robomaster app or run :ros:exec:`connect`:

  .. code-block:: console

    $ ros2 run robomaster_ros connect <SSID> <PASSWORD>

  Point the robot's camera towards the QR code and press the small button near the intelligent controller slider.
  The robot will read the code and connect to the network.


You can check that a robot is available and ready to be connected by running :ros:exec:`discover`:

.. code-block:: console

  $ ros2 run robomaster_ros discover
  [INFO] [1658231475.202434272] [discover]: Discovered 1 robots
  [INFO] [1658231476.378546964] [discover]: Connected to robot XXXXXXXXXXXXXX at 192.168.1.136: version 01.01.1131, battery 20%

The command will list the robots, which will beep and blink their LEDs.


You are ready to start controlling the robot through ROS:

.. code-block:: console

  $ ros2 launch robomaster_ros main.launch model:=<ep|s1>

Pick your robot type in ``model:=<ep|s1>`` to publish the correct URDF model and enable the appropriate modules.
