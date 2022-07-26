.. ros:currentnode:: RoboMasterROS

===
Arm
===

The robotic arm is controlled by two servo motors and features a parallel mechanism
(see :ros:model:`robomaster_description/arm.urdf.xacro`)
that maintain the orientation of the end effector and make the gripper to always
stay parallel to the chassis.

This module wraps class :py:class:`rm:robomaster.robotic_arm.RoboticArm`
through which we can control the arm servo motors and get update about the end effector position.


The arm needs to be calibrated to work properly. Perform the short calibration procedure
in the Robomaster App that consists of fully retracting the arm: when calibrated the servos will
both report a value of 180 degrees to the SDK  and 0 degrees in the App, which corresponds to the default values
of :ros:param:`arm.left_motor.zero` and :ros:param:`arm.right_motor.zero`.


State estimation
----------------

Parameter :ros:param:`chassis.rate` control the rate at which the arm state is gathered and republished to ROS.

- End effector position (estimated onboard) from :py:meth:`rm:robomaster.robotic_arm.RoboticArm.sub_position`
  is republished to :ros:pub:`arm_position`. The accuracy of this estimation depends on the arm calibration
  procedure performed in the Robomaster App but not from the calibration parameters passed to ROS (see next point).

- Joint states from :py:meth:`rm:robomaster.servo.Servo.sub_servo_info`
  is republished to :ros:pub:`joint_states_p` with the state of three joints using

  - ``rod_joint`` (left motor, using :ros:param:`arm.left_motor.zero`,
    :ros:param:`arm.left_motor.angle`, :ros:param:`arm.left_motor.direction`)
  - ``arm_1_joint`` (right motor, using :ros:param:`arm.right_motor.zero`,
    :ros:param:`arm.right_motor.angle`, :ros:param:`arm.right_motor.direction`)
  - ``rod_3_joint`` (one of the passive joint, whose angle is computed from the arm parallel
    kinematics as the difference between right and left motor angles

  The other joints in :ros:model:`robomaster_description/arm.urdf.xacro` are dependable of these three and
  automatically updated by :ros:exec:`robot_state_publisher/robot_state_publisher`.


Control
-------

The only SKD native way to *directly* control the arm is through an action :ros:act_server:`move_arm`
to reach a target goal position for the end effector.
Unfortunately, there is no way to configure the target speed during the action
and the robot does not offer end effector velocity control. The only way to move the end effector slowly, is to use a sequence of interpolated
position towards the target (a functionality not exposed by the ROS driver).

.. important::

  The arm can be *indirectly* controlled providing target angles or target angular speeds for the two servos.
  In fact, there are two mutually exclusive ways to control the arm:
  - arm related SDK commands from :py:class:`rm:robomaster.robotic_arm.RoboticArm`
  - servo related SDK commands from :py:class:`rm:robomaster.servo.Servo`.
  Arm commands are enabled if the arm has been configured in the Robomaster App,
  else servo commands are enabled.

  This fact is reflected in ROS: the ROS control commands described here are only effective
  if the arm has been configured.
  .. As this information is not available to SDK clients, users are responsible to pick the correct interfaces.


We also expose an stateless interface to send a relative target point ignoring action feedback
and states using subscription :ros:sub:`cmd_arm`.


Parameters
----------


.. ros:parameter:: arm.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

.. ros:parameter:: arm.rate int
  :default: 10
  :dynamic:

  The rate in Hz at which :ros:pub:`arm_position` and :ros:pub:`joint_states_p`
  (with arm joints) are published.
  One of ``0, 1, 5, 10, 20, 50`` (else it will approximate to the nearest value).
  A value of ``0`` will effectively disable updates.

.. ros:parameter:: arm.right_motor.zero int
  :default: 1024
  :dynamic:

  The raw right servo value when the arm is in the zero configuration. 1024 corresponds to 180 degrees.

.. ros:parameter:: arm.left_motor.zero int
  :default: 1024
  :dynamic:

  The raw left servo value when the arm is in the zero configuration. 1024 corresponds to 180 degrees.

.. ros:parameter:: arm.right_motor.angle float
  :default: -0.274016
  :dynamic:

  The right servo joint (``arm_1_joint``) position when the arm is in the zero configuration.
  When zero configuration has the arm fully retracted, it corresponds to the lower limit of ``arm_1_joint``.

.. ros:parameter:: arm.left_motor.angle float
  :default: 0.073304
  :dynamic:

  The left servo joint (``rod_joint``) position when the arm is in the zero configuration.

.. ros:parameter:: arm.right_motor.direction int
  :default: -1
  :dynamic:

  The orientation of the right motor with respect to the joint (``arm_1_joint``).

.. ros:parameter:: arm.left_motor.direction int
  :default: 1
  :dynamic:

  The orientation of the left motor with respect to the joint (``rod_joint``).


Subscriptions
-------------

.. ros:subscription:: cmd_arm geometry_msgs/Vector3

  Listen for relative target position commands for ``end_point_link`` in
  :ros:model:`robomaster_description/arm.urdf.xacro`), which are passed to the robot
  without using SDK actions. This provides a similar interface to :ros:act_server:`move_arm`
  but able to update the target position without having to cancel the action.
  You can you this interface to continuously control the end effector cartesian position or velocity.

  .. warning::

    This interface is experimental. To be safe, use :ros:act_server:`move_arm`.

Publishers
----------

.. ros:publisher:: arm_position geometry_msgs/PointStamped

  Publishes the position, estimated onboard, of the arm end effector
  (``end_point_link`` in :ros:model:`robomaster_description/arm.urdf.xacro`) in the ``arm_base_link`` frame.
  If accuracy is low, redo the arm calibration procedure in Robomaster App.


Action Servers
---------------

.. ros:action_server:: move_arm robomaster_msgs/MoveArm

  Move the end effector towards a target position, which can be absolute (i.e., with respect to arm base)
  or relative (i.e., with respect to the current arm position).
  If speed is too high, try interpolating the target position.

  The control is performed on-board, most probably using inverse kinematics (i.e., no motion planning seems involved).
  Target position may not be reachable by the controller even when feasible. To overcome this limitation,
  you may plan a path in cartesian space and then pass waypoints to this action.

  .. warning::

    New goals are not accepted if a current goal is active. To change goal, you first need
    to `cancel <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.cancel_goal_async>`_
    the current goal.
