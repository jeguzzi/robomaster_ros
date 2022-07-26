============
Introduction
============

Robomaster
----------

`DJI Robomaster`_ is a family commercial, educational robots, which currently features
three types of ground robots (`S1`_, `EP Core`_, and EP) and one drone (TT).
Our tools covers only the Robomaster ground robots, which are effectively different configurations
of the same hardware parts. They all share the four-wheeled omnidirectional chassis, the battery, and
the camera. All robots have two controllers, connected among themselves and to other parts
(servos, wheel motors, hit sensors, LEDs) through a CAN bus:

Motion controller
  A micro-controller for low-level control that directly connects with the wheels, the chassis LEDs,
  and the sensors. It features 1 UART, 6 PWM, and 1 SBUS interfaces. It is powered from the battery.

Intelligent controller
  A micro-processor running Android for high-level control. It interfaces with clients such as
  the DJI Robomaster App
  (available for `mobile <https://www.dji.com/ch/downloads/djiapp/robomaster>`_
  and `PC <https://www.dji.com/ch/downloads/softwares/robomaster-win>`_)
  and the `official Python remote client library <https://github.com/dji-sdk/RoboMaster-SDK>`_.
  It manages the external communication with the robot (Wi-Fi and USB).
  It controls the camera via USB, the speaker, and perform machine vision processing.
  It is connected with the motion controller via CAN bus and powered from it.


The different product configurations are

S1
  The original product with a gimbal and a blaster mounted on top of the chassis.
  The camera is mounted on the gimbal.

EP Core
  EP (for *educational pack*) Core replaces the gimbal with a parallel mechanism arm and a gripper.
  The camera is mounted on the arm.
  It has a metal extension to the plastic chassis to mount various other parts,
  such ToF sensors and IO adapters.

EP
  It adds gimbal and blaster to EP Core. With EP, you can effectively recreate an S1 by mounting the gimbal
  (DJI "warrior" configuration) or an EP core by mounting the arm (DJI "engineer" configuration).
  Except from gimbal and blaster, it has the same components as the EP Core.


All three product are meant to be controlled by DJI Robomaster App, which let users:

- check the system state
- configure the robot parts (e.g., mounting/dismounting the arm)
- configure the Wi-Fi communication
- update the firmware
- program the robot using Python or Blockly (which gets transliterated to Python).
  Python scripts are executed *onboard* by a restricted Python runtime.
- teleoperate the robot

In addition, EP and EP Core, officially support a remote client Python library, which uses a binary
protocol to get state, send commands, and trigger actions on the robot. We provide a ROS client that wraps this client library,

.. note::

  The :doc:`onboard Python library <rm:python/apis>` used from the Robomaster App
  is different than the :doc:`remote Python client library <rm:python_sdk/modules>`.
  The first is interpreted by an onboard Python runtime. The second is interpreted on the user PC
  and communicates with the robots through IP sockets. The exposed functionality overlaps but it's not identical.
  :doc:`DJI official documentation <rm:index>` distinguishes between them using the term SDK for the remote client library.

.. _DJI Robomaster: https://www.dji.com/ch/products/steam
.. _S1: https://www.dji.com/ch/robomaster-s1
.. _EP Core: https://www.dji.com/ch/robomaster-ep-core
.. _DJI Robomaster App: https://www.dji.com/ch/downloads/djiapp/robomaster
.. _SDK: https://github.com/dji-sdk/RoboMaster-SDK

ROS2 and Robomaster
-------------------

Three packages provides the core functionality to use Robomaster robots in ROS2:

:ros:pkg:`robomaster_msgs`
  defines all custom ROS interfaces. When possible, we use already standard interfaces
  (like :ros:msg:`geometry_msgs/Twist` for velocity commands) to favor integration with existing ROS tools
  and libraries. Custom interfaces are needed to exposes all functionality offered by the robot
  (e.g., :ros:msg:`ArmorHit` wraps all information about hit events) and to describe specific features
  (e.g., the id of LED in :ros:msg:`LEDEffect`).

:ros:pkg:`robomaster_description`
  contains ``urdf`` and ``xacro`` files to model the various component of the robots and
  the different way to configure a robot with them (i.e., EP and S1).
  The models have realistic visual and physical (i.e, inertia and collision shapes) details and
  can be used for state estimation, for simulation, and for visualization.

:ros:pkg:`robomaster_ros`
  contains Python libraries and executables to connect and control the robot. In particular, executable
  :ros:exec:`robomaster_driver` offers a ROS2 driver for the robot, wrapping the official client library,
  that exposes *all* available functionalities related to the robot's parts.
  Users can configure the driver to use a subset of parts to reduce computational and networking costs.


.. important::

  The wrapping of the client library in ROS follows these principles:

  Completeness
    All available functionality is exposed.
    This sometimes requires to patch the client library to expose hidden information.

  Transparency
    We provide a *minimal* wrapper that maps directly to the client library API.
    That is, we don't add functionality that is not already present in the API:
    such extensions are delegated to client nodes.
    The client library provides three kind of interfaces, which are mapped one to one to ROS2:

    SDK actions to ROS2 actions
      The SDK uses actions for long running tasks, which maps very well with ROS2 actions.
      Contrary to ROS2 actions, there is not API to preempt SDK actions but stopping an action is done
      indirectly by sending control commands. All SDK actions provides
      the same feedback (progress estimation in percent), which we forward to ROS2.

    SDK commands to ROS2 subscribers
      Controlling the robot is done in the SDK by sending commands.
      We trigger commands when we received a message from ROS2 subscription.

    SDK "DDS" subscriber to ROS2 publishers
      Getting information from the robot is mostly done in the SDK by starting a subscriber at a fixed frequency.
      We forward the notifications to ROS2 publishers.

  Conformity
    Exception to the above transparency rule comes from the goal to be as conform as possible to ROS best practices.
    Therefore, we use SI units, geometric information uses ROS frame conventions, and
    information from several SDK subscribers is grouped to use common ROS interfaces
    such as :ros:msg:`nav_msgs/Odometry`. We try our best to offer a coherent API,
    sometimes overcoming limitations in the SDK (e.g., gripper control in ROS2 uses actions
    as the gripper takes some time to close and open, while the SDK uses an instantaneous command,
    providing no explicit feedback for the progress).
