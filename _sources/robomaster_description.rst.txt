======================
robomaster_description
======================

.. ros:package:: robomaster_description
  :summary:

  This package provides the URDF model of Robomaster robots.
  Hardware parts are modelled in separated xacro files, which are then assembled in the EP Core and S1
  configurations. The models have realistic visual and physical (i.e, inertia and collision shapes) details and
  can be used for state estimation, simulation, and visualization.



Models
------

Parts
^^^^^

.. ros:model:: arm.urdf.xacro

  The robot arm

  .. ros:model-viewer:: arm.gltf


.. ros:model:: base.urdf.xacro

  The four-wheeled chassis

  .. ros:model-viewer:: base.gltf

.. ros:model:: camera.urdf.xacro

  The camera


.. ros:model:: extension.urdf.xacro

  The metallic chassis extension to mount parts


.. ros:model:: gimbal.urdf.xacro

  The gimbal and blaster

  .. ros:model-viewer:: gimbal.gltf

.. ros:model:: gripper.urdf.xacro

  The gripper

  .. ros:model-viewer:: gripper.gltf

.. ros:model:: intelligent_controller.urdf.xacro

  The intelligent controller


.. ros:model:: led.urdf.xacro

  One chassis LED (a submodel of :ros:model:`base.urdf.xacro`)


.. ros:model:: tof.urdf.xacro

  The Time-of-flight distance sensor


.. ros:model:: wheels.urdf.xacro

  One wheel (a submodel of :ros:model:`base.urdf.xacro`)



Robots
^^^^^^

.. ros:model:: robomaster_ep.urdf.xacro

  The assembled Robomaster EP Core with arm, gripper, and metallic extension.
  The intelligent controller is mounted in the extension. The camera in mounted on the arm.

  .. ros:model-viewer:: robomaster_ep.gltf

.. ros:model:: robomaster_s1.urdf.xacro

  The assembled Robomaster S1 with gimbal and blaster.
  The intelligent controller and the camera are mounted on the gimbal.

  .. ros:model-viewer:: robomaster_s1.gltf

Launch files
-------------

.. ros:autolaunch_file:: robomaster_description model.launch.py

    Evaluate the xacro model and passes it to :ros:exec:`robot_state_publisher/robot_state_publisher`.

.. ros:autolaunch_file:: robomaster_description main.launch

    Import :ros:launch:`main.launch` and launches :ros:exec:`joint_state_publisher/joint_state_publisher`.
    Use this launch file to launch everything needed for the robot model.
