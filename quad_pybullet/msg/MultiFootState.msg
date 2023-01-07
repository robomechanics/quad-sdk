# This is a message to hold the state of all feet of a legged robot
#
# The states of each foot are stored in a vector of FootState messages
# (0 = front left, 1 = back left, 2 = front right, 3 = back right).
# Accurate timing information is stored in the header

Header header
uint32 traj_index
quad_msgs/FootState[] feet
