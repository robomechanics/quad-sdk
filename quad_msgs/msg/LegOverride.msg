# This is a message of motor commands for overriding leg commands.
#
# Accurate timing information is stored in the header

Header header

uint8[] leg_index # 0: FL, 1:BL, 2:FR, 3:BR
LegCommand[] leg_commands
