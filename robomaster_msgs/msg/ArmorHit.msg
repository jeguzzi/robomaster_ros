# A hit event detected by one of the sensors placed behind the LEDs.
# The Robomaster detects two kind of hits: collision using microphones,
# and infrared beams using infrared receivers.

uint8 BOTTOM_BACK = 1   # sensor location: on back of the chassis
uint8 BOTTOM_FRONT = 2  # sensor location: on front of the chassis
uint8 BOTTOM_LEFT = 4   # sensor location: on left of the chassis
uint8 BOTTOM_RIGHT = 8  # sensor location: on right of the chassis
uint8 TOP_LEFT = 16     # sensor location: on left of the gimbal
uint8 TOP_RIGHT = 32    # sensor location: on right of the gimbal

uint8 COLLISION = 0     # type: detected by a microphone
uint8 IR = 1            # type: detected by the IR sensor

std_msgs/Header header  # header with the time stamp of the hit
uint8 type              # type of hit; one of ``COLLISION`` or ``IR``
uint8 location          # a bit mask for which sensors have detected the hit
uint16 level            # the hit impact strength as measured by the sensor microphone.
                        # Only applies to events of ``COLLISION`` type
