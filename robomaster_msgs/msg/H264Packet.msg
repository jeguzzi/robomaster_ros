# Encapsulate a packet from a H264 encoded video stream

std_msgs/Header header  # header with the time stamp and origin of the packet
uint64 seq              # sequential number of the packet
uint8[] data            # H264 data
