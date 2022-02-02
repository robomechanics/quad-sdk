# This is a message to hold a robot state
#
# The state is defined as an Odometry message with the body odometry, and a JointState message with joint positions and velocities
# Accurate timing information is stored in the header

Header header
uint32 traj_index
quad_msgs/BodyState body
sensor_msgs/JointState joints
quad_msgs/MultiFootState feet
