.. ros:currentnode:: RoboMasterROS

Gimbal
======

This module wraps the SDK class :py:class:`rm:robomaster.gimbal.Gimbal`
to get the state and control the gimbal.

If the module is enabled, the ROS driver when initializing will recenter the gimbal and
set the coupling control model between gimbal and chassis to :ros:param:`mode`.



State Estimation
----------------

Information from :py:meth:`rm:robomaster.gimbal.Gimbal.sub_angle` is republished
to :ros:pub:`joint_states_p` (joints ``gimbal_joint`` and ``blaster_joint``
of model :ros:model:`robomaster_description/gimbal.urdf.xacro`)
at the rate configured by parameter :ros:param:`gimbal.rate`.

Control
-------

The gimbal can be controlled in velocity, publishing target angular speeds on :ros:sub:`cmd_gimbal`,
or by giving a goal orientation (in specific coordinate frames)
to action :ros:act_server:`move_gimbal`, together with maximal rotation speeds.
Action :ros:act_server:`recenter_gimbal` recenter the gimbal.

To stop the gimbal, either send a zero velocity target or a zero relative target orientation,
To engage or disengage the gimbel motors, use :ros:sub:`gimbal/engage`.


Yaw control coupling mode between gimbal and chassis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The robot onboard controller can be configured to couple the controllers of gimbal and chassis yaws,
in one of three modes:

chassis is leading
  The gimbal will keep the same relative orientation with the chassis, effectively locking yaw rotations.

gimbal is leading
  The chassis will actively rotate to keep the same relative orientation.

free
  The two controllers are decoupled. When the chassis rotates,
  the gimbal (if no other commands are provided) will maintain absolute orientation,
  until it reaches the limits of yaw rotations.

The initial coupling mode is configured by the value of parameter ::ros:param:`mode`. To change mode,
send a command to :ros:sub:`mode`.

.. warning::

  Gimbal control commands will change the coupling mode: velocity commands on :ros:sub:`cmd_gimbal` will
  stop making the gimbal follows the chassis.

..
  TODO: same with actions?

Command :ros:sub:`gimbal/lock` lock the gimbal to the chassis, freezing both yaw and pitch rotations.

..
  TODO: check that this is why are we using lock AND set_mode?
  I.e., to lock pitch too because yaw should already be handled by set_mode


Parameters
----------

.. ros:parameter:: gimbal.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.


.. ros:parameter:: mode int
  :default: 2

  A value between 0 and 2, see the enum in :ros:msg:`robomaster_msgs/Mode`:
  0 free, 1 gimbal leads, 2 chassis leads.

.. ros:parameter:: gimbal.rate int
  :default: 10
  :dynamic:

  The rate in Hz at which :ros:pub:`joint_states_p` (with gimbal joints) is published.
  One of ``0, 1, 5, 10, 20, 50`` (else it will approximate to the nearest value).
  A value of ``0`` will effectively disable updates.


Subscription
------------

.. ros:subscription:: cmd_gimbal robomaster_msgs/GimbalCommand
  :qos-reliability: reliable
  :qos-durability: volatile

  Listen for velocity commands to control yaw and pitch speed of the gimbal using
  :py:meth:`rm:robomaster.gimbal.Gimbal.drive_speed`.

.. ros:subscription:: mode robomaster_msgs/Mode
  :qos-reliability: reliable
  :qos-durability: volatile

  Listen to commands to change the yaw coupling mode between gimbal and chassis.

.. ros:subscription:: gimbal/lock std_msgs/Bool
  :qos-reliability: reliable
  :qos-durability: volatile

  Listen for commands to lock the gimbal to the frame (when ``msg.data`` is true).

.. ros:subscription:: gimbal/engage std_msgs/Bool
  :qos-reliability: reliable
  :qos-durability: volatile

  Listen for commands to engage or disengage the gimbal motors.

..
  Service Servers
  ---------------

  .. ros:service_server:: set_status std_srvs.srv.SetBool
    :qos-reliability: reliable
    :qos-durability: volatile

Action Servers
---------------

.. ros:action_server:: move_gimbal robomaster_msgs/MoveGimbal

  Pass a target orientation in one of the 4 coordinate frame described
  in :ros:action:`robomaster_msgs/MoveGimbal` to an onboard action,
  which terminates when the gimbal reaches the goal.

..
  TODO: check the coupling state after the action

.. ros:action_server:: recenter_gimbal robomaster_msgs/RecenterGimbal

  Recenter the gimbal using onboard control triggered by a different SDK action
  :py:meth:`rm:robomaster.gimbal.Gimbal.recenter`.
  The action terminates when the gimbal reaches zero yaw and pitch with respect to the chassis.

.. warning::

  The two action are mutually exclusive: the ROS driver avoid triggering both in parallel.
  Moreover, new action goals are not accepted if a current goal is active. Therefore,
  to call a different action or to change goal, you first need
  to `cancel <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.cancel_goal_async>`_
  the current goal.
