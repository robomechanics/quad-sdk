<<<<<<< HEAD:quad_msgs/msg/LegCommand.msg
# This is a message of motor commands for each joint on a quad leg.
=======
# This is a message of motor commands for each joint on a quad leg
>>>>>>> Switch build system to catkin_tools, switch spirit* to quad*:spirit_msgs/msg/LegCommand.msg
#
# Accurate timing information is stored in the header

Header header

MotorCommand[] motor_commands # Stored as Abd, Hip, Knee
