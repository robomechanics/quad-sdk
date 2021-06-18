# This is a message to hold an array of ground reaction forces and their points of application
#
# Accurate timing information is stored in the header.

Header header
geometry_msgs/Vector3[] vectors
geometry_msgs/Point[] points
bool[] contact_states
uint32 traj_index