# This is a message to hold a robot plan
#
# The plan is defined as an array of odometry messages.
# Accurate timing information for localization is stored in the header.
# This should match the first state in the states vector.

std_msgs/Header header
builtin_interfaces/Time global_plan_timestamp
builtin_interfaces/Time state_timestamp
quad_msgs/RobotState[] states
quad_msgs/GRFArray[] grfs
uint32[] plan_indices
uint32[] primitive_ids
float64 compute_time
quad_msgs/RobotPlanDiagnostics diagnostics
