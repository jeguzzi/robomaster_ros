# High-level chassis state provided by the robot through :py:meth:`robomaster.chassis.Chassis.sub_status`
# TODO(Jerome): check


std_msgs/Header header  # header with the time stamp of the update
bool is_static          # whenever the robot is still
bool up_hill            # whenever the robot is facing uphill
bool down_hill          # whenever the robot is facing downhill
bool on_slope           # whenever the robot is on a lateral slope
bool is_pick_up         # whenever the robot has been picked up
bool slip               # whenever wheels are slipping
bool impact_x           # whenever the accelerometer has registered a collision longitudinally
bool impact_y           # whenever the accelerometer has registered a collision laterally
bool impact_z           # whenever the accelerometer has registered a collision vertically
bool roll_over          # whenever the robot is capsized
bool hill               # whenever the robot is facing uphill and is still.
