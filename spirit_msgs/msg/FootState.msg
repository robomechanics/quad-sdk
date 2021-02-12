# This is a message to hold the state of a single foot of a legged robot
#
# The states of each foot are stored in a vector of FootState messages
# Accurate timing information is stored in the header

Header header
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
bool contact