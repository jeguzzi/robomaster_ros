======================
Robomaster Driver Node
======================

.. ros:node:: RoboMasterROS
  :summary:

  This node provides a ROS interface to a Robomaster using the official Python client library.

  It wraps functionality from :py:class:`rm:robomaster.robot.Robot` to create and monitor a connection to a robot,
  configured by parameters :ros:param:`conn_type`, :ros:param:`serial_number`, and :ros:param:`reconnect`.

The robot and the client library are modulars: for example, one can attach a ToF sensor
and use class :py:class:`rm:robomaster.sensor.DistanceSensor` to read it.
The ROS node follows the same organization, with submodules that corresponds to hardware parts.
Modules can be enabled using parameters :ros:param:`<module>.enabled`: by default they are all disabled.


Modules
^^^^^^^

.. toctree::
  :glob:

  robomaster_ros_modules/*


Parameters
^^^^^^^^^^

The following parameters configures the client library and the general behavior of the ROS node.

.. ros:parameter:: lib_log_level string
  :default: "ERROR"

  The logging level used for the internal SDK wrapped by the ROS node (see :doc:`rm:python_sdk/log`).

.. ros:parameter:: conn_type string
  :default: "sta"

  The connection method between client and robot (see :doc:`rm:python_sdk/connection`), one of:

  ``"sta"``
    through a managed Wi-Fi network (SSID and password needs to be communicated to the robot,
    for instance using `connect`)

  ``"ap"``
    through the robot's Wi-Fi access point (SSID and password are printed on the intelligent controller)

  ``"rndis"``
    through a network USB interface (micro USB port on the side of the intelligent controller)

.. ros:parameter:: serial_number string
  :default: ""

  If non empty, it is passed to :py:meth:`robomaster.robot.Robot.initialize` to select which robot to connect.
  The serial number is a 14-long code printed on the intelligent controller.
  If the parameter value is a shorter, it's end is padded by ``*``, while if it is longer, it is truncated.

  .. note:: Shorter serial numbers are useful for simulated robot.


.. ros:parameter:: reconnect bool
  :default: true

  If enabled, will keep the ROS node on while waiting for a robot to reconnect.
  In disabled, the node will terminate once the robot disconnects.


.. ros:parameter:: tf_prefix string
  :default: ""

  The prefix that gets prepended, if not empty, to any frame related to the robot,
  e.g., ``base_link`` -> ``<tf_prefix>/base_link``.

  .. warning::

    You need to pass the same ``tf_prefix`` to the robot model too.
    Launch file :ros:launch:`robomaster_ros/main.launch` does it automatically.


Publishers
^^^^^^^^^^

The following publishers are shared between modules

.. ros:publisher:: joint_states_p sensor_msgs/JointState
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes partial updates about joints: :ref:`wheels <chassis state>`, gimbal, arm, and gripper.

  .. note::

    :ros:launch:`robomaster_ros/main.launch` configures ``joint_state_publisher`` to group
    together :ros:pub:`joint_states_p` and republish them on
    ``join_states``, which is used by ``robot_state_publisher`` to update the model.


.. ros:publisher:: /tf tf2_msgs/TFMessage
  :qos-reliability: reliable
  :qos-durability: volatile

  This publisher is used indirectly to broadcast the transform :ref:`odom -> base_link <chassis state>`.
