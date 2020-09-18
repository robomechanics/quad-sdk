# This is a message to hold a single spirit footstep
#
# A footstep is defined by the leg index, the point of contact,
# and the touchdown time and stance duration. Touchdown time is
# expressed as a duration from the beginning of localized time.

uint32 index
geometry_msgs/Point position
duration td
duration ts