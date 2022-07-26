# Raw audio samples
# .. note::
#   Unlike ROS1, ROS2 has no ``audio_common_msgs``, so for now we define our own format
#   to publish the raw 16-bit 48Khz audio stream from the camera mono microphone

std_msgs/Header header  # the timestamp and frame of the microphone recording
int16[] data            # an array of 16-bit audio samples
