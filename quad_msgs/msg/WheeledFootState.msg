# This is a message to hold the state of a single foot of a wheeled-legged robot
#
# The states of each foot are stored in a vector of WheeledFootState messages
# Accurate timing information is stored in the header

std_msgs/Header header
uint32 traj_index
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool contact
float64 wheel_angle
float64 wheel_velocity
float64 wheel_torque