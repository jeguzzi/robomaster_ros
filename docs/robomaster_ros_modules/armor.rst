.. ros:currentnode:: RoboMasterROS

Armor
=====
The Robomaster detects two kind of hits: collision using microphones,
and infrared beams using infrared receivers.

The four microphones are placed behind the chassis LEDs.
They are designed to detect small balls fired by another Robomaster blaster.
Their sensitivity is configured at launch time using parameter :ros:param:`armor.sensitivity`.

The two infrared sensors are placed on the two sides of the gimbal.
They are designed to detect IR beams emitted by another Robomaster blaster.

Each time one of the sensors detect a hit,
we forward the information from :py:meth:`robomaster.armor.Armor.sub_hit_event`
to the topic :ros:pub:`hit`.

The ROS driver wraps the SDK module :py:mod:`robomaster.armor`.

.. note::

  We patched :py:meth:`robomaster.armor.ArmorHitEvent.data_info` to include the hit strength,
  which we then pass to the message.

.. TODO(Jerome): Complete once tested.
   Include a description of the IR sensors/emitters from the manual.

Parameters
----------

.. ros:parameter:: armor.enabled bool
  :default: false


.. ros:parameter:: armor.sensitivity float
  :default: 0.5
  :dynamic:

  The relative detection threshold between 0 and 1 (in 0.1 steps) for all four collision sensors.
  Lower the value to increase sensitivity.


Publishers
----------

.. ros:publisher:: hit robomaster_msgs/ArmorHit
  :qos-reliability: reliable
  :qos-durability: volatile

  Publishes a message each time a hit is detected. The message is stamped when it is published.
  The message contains the sensor that detected the hit, the type of hit (collision or infrared beam),
  and the raw strength in case of a collision.
