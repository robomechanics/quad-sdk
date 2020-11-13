# This is a message to hold the desired position, desired velocity, feedforward torques and control gains for a single joint on Spirit
#
# Accurate timing information is stored in the header

Header header

float64 pos_setpoint # Position command
float64 vel_setpoint # Velocity command
float32 kp # Position setpoint gain
float32 kd # Derivative setpoint gain
float64 torque_ff # Feedforward torque
