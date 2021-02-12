# This is a message to hold a discrete foot plan for multiple robot feet
#
# The plans of each foot are stored in a vector of FootPlanDiscrete messages
# (0 = front left, 1 = back left, 2 = front right, 3 = back right).
# Accurate timing information to localize the plans is stored in the header

Header header
spirit_msgs/FootPlanDiscrete[] feet