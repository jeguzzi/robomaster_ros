# Coupling control model between gimbal and chassis.

uint8 FREE = 0          # mode: gimbal and chassis are decoupled
uint8 GIMBAL_LEAD = 1   # mode: chassis follows gimbal movements
                        # to maintain the same relative orientation
uint8 CHASSIS_LEAD = 2  # mode: gimbal follows gimbal movements
                        # to maintain the same relative orientation

uint8 mode              # coupling mode, one of ``FREE``, ``GIMBAL_LEAD``, ``CHASSIS_LEAD``
