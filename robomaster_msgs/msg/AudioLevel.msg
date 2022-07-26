# Sound level
# .. note::
#   Unlike ROS1, ROS2 has no ``audio_common_msgs``, so [for now] we define our own format
#   for sound level in dBFS

std_msgs/Header header  # the timestamp and frame of the microphone recording
float32 level           # sound level in dBFS
