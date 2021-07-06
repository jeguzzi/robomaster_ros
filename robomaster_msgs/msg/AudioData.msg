# ROS2 has no audio_common[_msgs], so for now we define our own format
# to publish the raw 16-bit 48Khz audio stream from the camera mono microphone
std_msgs/Header header
int16[] data
