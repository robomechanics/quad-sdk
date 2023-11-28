<<<<<<< HEAD:quad_msgs/msg/LegCommandArray.msg
# This is a message of leg commands for each leg on quad. The order is in FL, BL, FR, BR.
=======
# This is a message of motor commands for each leg on quad
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_msgs/msg/LegCommandArray.msg
#
# Accurate timing information is stored in the header

Header header

LegCommand[] leg_commands # FL, BL, FR, BR
