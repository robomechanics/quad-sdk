# This is a message of motor commands for each leg on spirit
#
# Accurate timing information is stored in the header

Header header

LegCommand[] leg_commands # FL, BL, FR, BR
float64 dt_first_step
bool[] miss_contact