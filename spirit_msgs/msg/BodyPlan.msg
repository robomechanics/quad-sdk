# This is a message to hold a spirit body plan
#
# The plan is defined as an array of odometry messages.
# Accurate timing information for localization is stored in the header.
# This should match the first state in the states vector.

Header header
uint32[] primitive_ids
nav_msgs/Odometry[] states
geometry_msgs/Vector3[] grfs
