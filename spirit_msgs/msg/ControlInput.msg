# This is a message to hold a spirit control input
#
# The control input is defined as the joint positions, joint velocities, body position, body velocity, body orientation, body angular velocity
# Accurate timing information is stored in the header

Header header
float32[] pos
float32[] vel
float32[] torque