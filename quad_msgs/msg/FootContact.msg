# Per-foot contact reading from the Unitree foot force sensor.
# Order is quad-sdk leg order: leg_0=FL, leg_1=RL, leg_2=FR, leg_3=RR.
# foot_force_raw is the int16 reading directly from LowState.foot_force().
# contact_states is a thresholded boolean (true == foot loaded).

std_msgs/Header header
bool[] contact_states
int16[] foot_force_raw
