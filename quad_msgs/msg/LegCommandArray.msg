# This is a message of leg commands for each leg on quad. The order is in FL, BL, FR, BR.
#
# Accurate timing information is stored in the header

std_msgs/Header header

LegCommand[] leg_commands # FL, BL, FR, BR
