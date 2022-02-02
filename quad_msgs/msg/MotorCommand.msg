# This is a message to hold the desired position, desired velocity, feedforward torques and control gains for a single joint on Quad
#
# Accurate timing information is stored in the header

Header header

# Commands
float64 pos_setpoint # Position command
float64 vel_setpoint # Velocity command
float32 kp # Position setpoint gain
float32 kd # Derivative setpoint gain
float64 torque_ff # Feedforward torque

# Diagnostics
float64 pos_component # Feedback position component
float64 vel_component # Feedback velocity component
float64 fb_component # Feedback total component
float64 effort # Total effort
float64 fb_ratio # Feedback to total ratio
