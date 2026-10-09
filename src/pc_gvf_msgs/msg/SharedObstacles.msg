# World-frame occupied evidence only. This message never certifies free space.
std_msgs/Header header
string robot_id
string session_id
uint64 sequence
bool overloaded
geometry_msgs/Point[] centers
geometry_msgs/Vector3[] velocities
float64[] radii
float64[] observed_at
int32[] observation_counts
