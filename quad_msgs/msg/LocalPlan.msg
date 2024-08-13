# This is a message to hold a quad local plan
#
# The plan is defined as an array of odometry messages.
# Accurate timing information for localization is stored in the header.
# This should match the first state in the states vector.

std_msgs/Header header
quad_msgs/RobotState[] states
quad_msgs/GRFArray[] grfs
uint32[] plan_indices
builtin_interfaces/Time global_plan_timestamp