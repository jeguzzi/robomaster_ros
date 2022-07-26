==============
robomaster_ros
==============

.. ros:currentnode:: RoboMasterROS

.. ros:package:: robomaster_ros
  :summary:


Executables
-----------

.. ros:executable:: connect [-h] [--app_id APP_ID] ssid password

  The program shows a QR code that encodes SSID and password:
  point the robot's camera towards the QR code and
  press the small button near the intelligent controller slider.
  The robot will read the code and try to connect to the network.
  The program wait until the robot connects and then terminates.

  :arg ssid: The network SSID
  :arg password: The network password
  :opt app_id: The app id of at most 7 letters

.. ros:executable:: discover [-h] [--timeout TIMEOUT] [--modules]

  The command lists the available robots with their serial number, IP address,
  firmware version, and battery level. Discovered robots will beep and blink their LEDs.

  :opt timeout: How much to wait for robots
  :opt modules: Probe the robot modules

.. ros:executable:: robomaster_driver

  Execute a Robomaster ROS driver instantiating :ros:node:`RoboMasterROS`. When the robot disconnects,
  it waits until the robots connects again when :ros:param:`RoboMasterROS:reconnect` is enabled
  (destroying and creating another node each time).
  It tries to shut down the node as graceful as possible, stopping any ongoing action
  before notifying modules to stop.

  To limite the ROS driver to just module <module>, run

  .. code-block:: console

    ros2 run robomaster_ros robomaster_driver --ros-args --param <module>.enabled:=true

  .. important:: Execute one driver for each robot you want to control using different namespaces.


.. ros:executable:: h264_decoder

  TODO

.. ros:executable:: display_battery

  TODO

.. ros:executable:: play_audio

  TODO


Launch files
------------

.. ros:autolaunch_file:: robomaster_ros main.launch
