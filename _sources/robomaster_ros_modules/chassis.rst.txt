.. ros:currentnode:: RoboMasterROS

Chassis
========

The ROS driver wraps the module :py:mod:`robomaster.chassis` related to the wheels and the chassis state.
The module is enabled by :ros:param:`chassis.enabled`.

The Robomaster has 4 independently controllable `mecanum wheels <https://en.wikipedia.org/wiki/Mecanum_wheel>`_,
with encoders, and an IMU sensor (accelerometer and magnetometer).


.. _chassis state:

Sensing and State Estimation
----------------------------

It gather the onboard state estimation from several SDK subscribers
at a rate controlled by parameter :ros:param:`chassis.rate`:
- orientation from :py:meth:`robomaster.chassis.Chassis.sub_attitude`
- wheels' angular position and velocity from :py:meth:`robomaster.chassis.Chassis.sub_esc`
- angular velocity and linear acceleration from :py:meth:`robomaster.chassis.Chassis.sub_imu`
- horizontal pose from :py:meth:`robomaster.chassis.Chassis.sub_position`
- horizontal velocity from :py:meth:`robomaster.chassis.Chassis.sub_velocity`

merges it and republishes to ROS as

- fused odometry estimation in :ros:pub:`odom`
- fused orientation (only if :ros:param:`chassis.imu_includes_orientation` is enabled)
  and imu sensor readings in :ros:pub:`imu`
- wheels' joint state in :ros:pub:`joint_states_p`
- a tf transform from ``odom`` to ``base_link`` from the pose component of the odometry

Use parameter :ros:param:`chassis.force_level` to force the published state estimation
to be perfectly horizontally, effectively projecting the orientation to be parallel to the ground.
In any case, the robot is not tracking the vertical position, which is always set to zero.
Enable parameter :ros:param:`chassis.odom_twist_in_odom` to switch to the odom frame for the twist.

The driver also publishes the chassis state on topic :ros:pub:`state` gathered from :py:meth:`robomaster.chassis.Chassis.sub_status`
at a rate defined by parameter :ros:param:`chassis.status.rate`.
The state contains macroscopic information about the robot: if it is moving, if it is leveled, ... .

Control
-------

Controlling mecanum wheels, it is possible to move the robot horizontally in any direction,
independently of the angular speed. The wheel motors can be engaged or disengaged using service :ros:srv_server:`engage_wheels`.
At launch, wheels are engaged but don't resist movements. After the robot applies the first control command (see below),
the wheel will resist movements even when stopped. To let them free, you need to disengage them,
which will also preempt any current control.

.. attention::

  If you use  :ros:srv_server:`engage_wheels` as an emergency stop,
  be aware that the robot will slide if it was moving or when on a slope.

To move the robot:

  - send target linear wheel speeds to :ros:sub:`cmd_wheels` to control individual wheels.
    Wheel speeds are forwarded through :py:meth:`robomaster.chassis.Chassis.drive_wheels` and limited to 3.5 m/s.

  - send a target twist to :ros:sub:`cmd_vel` to control the robot omni-directionally on the floor.
    When parameter :ros:param:`chassis.twist_to_wheel_speeds` is enabled,
    the ROS driver computes the appropriate target wheel speeds and forwards them to the robot:
    When the parameter is not enabled, the target twist is directly forwarded
    through :py:meth:`robomaster.chassis.Chassis.drive_speed` (i.e., the twist control is done onboard).

  - send a goal pose (relative to the current robot pose) to :ros:act_server:`move`. The robot will then move towards the goal,
    providing feedback on the progress. Use this action if you want to control the robot in space when the goal is fixed.

The SDK velocity commands have an optional deadman check controlled by parameter :ros:param:`chassis.timeout`.
When enabled, it makes the robot stops if too much time passes from the last command.   If the value is larger than 0, it is passed as timeout time when calling SDK commands

.. attention::

  Controlling the velocity through target wheel speeds or chassis twist impacts the robot's behavior.

  target wheel speed
    The robot ignores the chassis state estimation and just tries to set the target speed.
    Once stopped, if you manually (i.e., pushing it or lifting it) rotate the chassis, the robot won't react more
    than resist wheel movements. The robot's speed is maximal in this mode and you risk damaging it from collisions.

  target twist or pose
    The robot uses the chassis state estimation and tries to match the target estimation with the current estimation.
    Once stopped, if you manually (i.e., pushing it or lifting it) rotate the chassis,
    the robot will counter react to keep the same pose.
    Moreover, for Robomaster EP, the allowed target speeds are lower at 0.8 m/s,
    probably to protect the arm servos.


There are three ways to stop the robot. Once stopped the robot behaves differently.

  - Sending an zero twist to :ros:sub:`cmd_vel`: once stopped, the robot resists movements (see above). Would preempt an active :ros:act_server:`move` goal.

  - Sending an array with zero speeds to :ros:sub:`cmd_wheels`: once stopped,
    individual wheels resist movements (see above). Would preempt an active :ros:act_server:`move` goal.

  - Sending a zero goal to :ros:act_server:`move`: once stopped,
    the robot does not resist movements. Only effective if no goal are active.

  - Preempting a :ros:act_server:`move` goal: once stopped,
    the robot does not resist movements. Only effective if a goal is active.


Parameters
----------

.. ros:parameter:: chassis.enabled bool
  :default: false

  Enables all the ROS interface described here, which by default is disabled, like all other modules.

.. ros:parameter:: chassis.timeout float
  :default: 0.0
  :dynamic:

  Set the command deadline in seconds. Values less or equal to zero disable the deadman check.
  Positive values are passed as ``timeout`` to the SDK methods :py:meth:`robomaster.chassis.Chassis.drive_wheels`
  and :py:meth:`robomaster.chassis.Chassis.drive_speed`.

.. ros:parameter:: chassis.twist_to_wheel_speeds bool
  :default: false
  :dynamic:

  When enabled, topic :ros:sub:`cmd_vel` will control wheel state instead of chassis state.
  Useful to unlock full speed or to avoid that the robot tries to maintain orientation when lifted or pushed.

.. ros:parameter:: chassis.force_level bool
  :default: false
  :dynamic:

  When enabled, it force the published state estimation in :ros:pub:`odom` (and tf)
  to be horizontal.

.. ros:parameter:: chassis.odom_twist_in_odom bool
  :default: false
  :dynamic:

  When enabled, it switches the twist component of the published state estimation in :ros:pub:`odom`
  to the robot's ``odom`` frame. The default is instead to use ``base_link``, i.e.,
  to publish a twist relative to the robot.

.. ros:parameter:: chassis.rate int
  :default: 10
  :dynamic:

  The rate in Hz at which :ros:pub:`odom`, :ros:pub:`imu`, and  :ros:pub:`joint_states_p`
  (with wheel joints) are published.
  One of ``0, 1, 5, 10, 20, 50`` (else it will approximate to the nearest value).
  A value of ``0`` will effectively disable updates.

.. ros:parameter:: chassis.status.rate int
  :default: 1
  :dynamic:

  The rate in Hz at which :ros:pub:`state` is published.
  One of ``0, 1, 5, 10, 20, 50`` (else it will approximate to the nearest value).
  A value of ``0`` will effectively disable updates.

.. ros:parameter:: chassis.imu_includes_orientation bool
  :default: true
  :dynamic:

  When enabled, :ros:pub:`imu` will include the orientation from :ros:pub:`odom`.

.. ros:parameter:: chassis.error.linear_velocity.xy float
  :default: 0.005
  :dynamic:

  Horizontal linear velocity standard deviation in :ros:pub:`odom` in m/s

.. ros:parameter:: chassis.error.angular_velocity.xy float
  :default: 0.01
  :dynamic:

  Non-horizontal (i.e., roll and pitch components) angular velocity standard deviation
  in :ros:pub:`odom` and :ros:pub:`imu` in rad/s.

.. ros:parameter:: chassis.error.angular_velocity.z float
  :default: 0.03
  :dynamic:

  Horizontal angular (i.e., yaw component) velocity standard deviation
  in :ros:pub:`odom` and :ros:pub:`imu` in rad/s.

.. ros:parameter:: chassis.error.linear_acceleration.xyz float
  :default: 0.1
  :dynamic:

  Linear acceleration standard deviation in :ros:pub:`imu` in m/s^2.

Subscription
------------

.. ros:subscription:: cmd_vel geometry_msgs/Twist
  :qos-reliability: reliable
  :qos-durability: volatile

  Receives a target twist in robot frame (``base_link``).
  Only horizontal components (i.e., ``linear.x``, ``linear.y``, ``angular.z``) are considered.
  Values are clamped by the SDK before passing them to the robot.
  If :ros:param:`chassis.twist_to_wheel_speeds` is enabled, the message is converted to target wheel speeds, which are then
  treated as for :ros:sub:`cmd_wheels`.
  If :ros:param:`chassis.timeout` is strictly positive, the robot will stop after ``chassis.timeout``
  seconds if no more messages are received.

.. ros:subscription:: cmd_wheels robomaster_msgs/WheelSpeeds
  :qos-reliability: reliable
  :qos-durability: volatile

  Receives a list of target speeds, one for each wheel.
  Values are clamped by the SDK before passing them to the robot.
  If :ros:param:`chassis.timeout` is strictly positive, the robot will stopß after ``chassis.timeout``
  seconds if no more messages are received.

Publishers
----------

.. ros:publisher:: odom nav_msgs/Odometry
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes the most recent state estimation. Publishing happens synchronously to :ros:pub:`imu`,
  with which it shares orientation and angular velocity.
  Depending on the value of :ros:param:`chassis.odom_twist_in_odom`, pose and twist
  are in the same ``odom`` frame or the twist is in the ``base_link`` frame.
  Update rate is controlled by parameter :ros:param:`chassis.rate`.
  The publishing rate may be lower due to poor communication with the robot.

.. ros:publisher:: imu sensor_msgs/Imu
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes the most recent state estimation. Publishing happens synchronously to :ros:pub:`odom`,
  with which it shares orientation and angular velocity, with the addition of ``linear_acceleration`` measured by the accelerometer.
  Angular velocity is the fused value, not the value read by the gyroscope, which is not exposed by SDK.
  Update rate is controlled by parameter :ros:param:`chassis.rate`.
  The publishing rate may be lower due to poor communication with the robot.

.. ros:publisher:: state robomaster_msgs/ChassisStatus
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a compact state of the chassis.
  Update rate is controlled by parameter :ros:param:`chassis.status.rate`.
  Publishing rate may be lower due to poor communication with the robot.

Service Servers
---------------

.. ros:service_server:: engage_wheels std_srvs/SetBool

  Engage or disengage all motors at once.

Action Servers
---------------

.. ros:action_server:: move robomaster_msgs/Move

  Move the robot towards a target pose relative to the robot frame (``base_link``) at given
  angular and linear speed using an onboard controller.

  .. warning::

    New goals are not accepted if a current goal is active. To change goal, you first need
    to `cancel <https://docs.ros2.org/latest/api/rclpy/api/actions.html#rclpy.action.client.ClientGoalHandle.cancel_goal_async>`_
    the current goal.
