# This is a message to hold a spirit state estimate
#
# The state is defined as the joint positions, joint velocities, body position, body velocity, body orientation, body angular velocity
# Accurate timing information is stored in the header

Header header
float32[] body_pos
float32[] body_vel
float32[] body_ang_pos
float32[] body_ang_vel
sensor_msgs/JointState joint_states