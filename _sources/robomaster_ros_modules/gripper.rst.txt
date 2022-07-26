.. ros:currentnode:: RoboMasterROS

Gripper
=======

This module wraps the SDK class :py:class:`rm:robomaster.gripper.Gripper` to control the gripper using
the action :ros:act_server:`gripper`.

The SDK expose the macroscopic control state of the gripper: open, closed or paused
When "paused" the gripper is not actively trying to open or close. For example, if the gripper closes to grasp an object,
the state will switch to "closed". If not more commands are sent,
once the object is removed, the gripper will close more. On the contrary, if a "pause" target is sent,
the gripper won't close once the object is removed.

The state of the gripper changes due to discrete control actions.
Therefore we publish a new state in :ros:pub:`gripper` only when there is a change.

The module also publishes the state of all joints in
:ros:model:`robomaster_description/gripper.urdf.xacro` on :ros:pub:`joint_states_p`.
Joint positions are interpolated from a
pre-recorded kinematic simulation of the gripper, based on the current state.

.. warning::
  Because the actual opening of the gripper is not known, for computing joint states
  "open" and "closed" define a pose with the gripper *completely* open and closed, respectively.
  This leads to inaccuracies in the URDF model when the gripper is, in fact, partially open.

  Unfortunately there is not way to overcome this limitation within the SDK.
  One possible solution could be to use machine vision to infer the state of the gripper.

Parameters
----------

.. ros:parameter:: gripper.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

Publishers
----------

.. ros:publisher:: gripper robomaster_msgs/GripperState
  :qos-reliability: reliable
  :qos-durability: transient_local

  Publishes the gripper state each time it changes due to control actions.

Action Servers
---------------

.. ros:action_server:: gripper robomaster_msgs/GripperControl

  Sends a command with the target state and wait until the gripper reaches that state.
  When the target state is "paused", the action stop the gripper and return immediately.
  If the gripper does not reaches the target state before a timeout, the actions fails.
  The action feedback contains the current state while the response contains
  the duration of the action.

  .. note::
    This is the only ROS action that does not wrap an SDK action (as none is available in
    :py:class:`rm:robomaster.gripper.Gripper`).
