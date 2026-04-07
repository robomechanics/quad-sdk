# This is a message to hold a body contact force estimate
#
# The body contact force vector holds the estimated external torques acting on each of the 12 joints.
# Accurate timing information is stored in the header

std_msgs/Header header

float64[] joint_torques