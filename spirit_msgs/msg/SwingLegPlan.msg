# This is a message to hold a spirit swing leg plan
#
# The plans are defined as nav_msgs/Path variables by name (0 = front left, 1 = back left, 2 = front right, 3 = back right).
# Accurate timing information to localize the plans is stored in the header

Header header
# nav_msgs/Path fl
# nav_msgs/Path bl
# nav_msgs/Path fr
# nav_msgs/Path br
nav_msgs/Path[] legs