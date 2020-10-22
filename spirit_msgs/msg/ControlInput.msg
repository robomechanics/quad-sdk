# This is a message to hold a spirit control input
#
# The control input is defined as the desired ground reaction force for each leg.
# This information is sent to the low level controller onboard the robot for torque mapping.
# Accurate timing information is stored in the header.

Header header

geometry_msgs/Vector3[] grfs