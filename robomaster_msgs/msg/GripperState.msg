# State of the gripper

uint8 PAUSE=0            # Gripper inactive
uint8 OPEN=1             # Gripper open
uint8 CLOSE=2            # Gripper closed

std_msgs/Header header   # header with the time stamp of the update
uint8 state              # one of ``PAUSE``. ``OPEN``. or ``CLOSE``
