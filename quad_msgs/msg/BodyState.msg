# This is a message to hold a robot body state
#
# The body state is defined as an pose and twist messages
# Accurate timing information is stored in the header

Header header
uint32 traj_index
geometry_msgs/Pose pose
geometry_msgs/Twist twist
