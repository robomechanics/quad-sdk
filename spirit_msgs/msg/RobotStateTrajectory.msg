# This is a message to hold a trajectory of robot states
#
# States in the trajectory are stored in a vector of RobotState messages.
# The timing in each header must be accurately localized to the first state in the message.
# The header in this message should timestamp when the trajectory itself is published.

Header header
spirit_msgs/RobotState[] states