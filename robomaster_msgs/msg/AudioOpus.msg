# Encapsulate a packet from an OPUS encoded audio stream

std_msgs/Header header  # header with the time stamp and origin of the packet
uint64 seq              # sequential number of the packet
uint8[] buffer          # Opus data
