# This is a message to hold a body contact force estimate
#
# The body contact force is defined as the global-frame cartesian forces and torques estimated for each link of the robot body.
# Accurate timing information is stored in the header

Header header

geometry_msgs/Wrench body_wrench
geometry_msgs/Wrench flu_wrench
geometry_msgs/Wrench blu_wrench
geometry_msgs/Wrench fru_wrench
geometry_msgs/Wrench bru_wrench
geometry_msgs/Wrench fll_wrench
geometry_msgs/Wrench bll_wrench
geometry_msgs/Wrench frl_wrench
geometry_msgs/Wrench brl_wrench