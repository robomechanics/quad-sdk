# This is a message to hold a spirit local plan
#
# The plan is defined as an array of odometry messages.
# Accurate timing information for localization is stored in the header.
# This should match the first state in the states vector.

Header header
spirit_msgs/RobotState[] states
spirit_msgs/GRFArray[] grfs
