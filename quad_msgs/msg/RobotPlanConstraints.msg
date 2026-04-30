# Time-windowed pose constraints used to guide the global body planner away
# from another robot's plan during conflict-based search.
#
# Each parallel array entry describes one constrained pose (one position the
# constrained robot occupies, with the body yaw and the plan-relative time
# window during which it occupies that pose). The planning robot must avoid
# placing its body so that its oriented bounding box overlaps the constrained
# robot's oriented bounding box at the corresponding time.
#
# The four arrays must all have the same length.

float64[] pos_x       # World-frame x position of constrained body, m
float64[] pos_y       # World-frame y position of constrained body, m
float64[] pos_z       # World-frame z position of constrained body, m
float64[] yaw         # Body yaw of constrained body, rad
float64[] t_start     # Start of forbidden time window (plan-relative, s)
float64[] t_end       # End of forbidden time window (plan-relative, s)

# Body extents of the constrained robot. Used to build its oriented bounding
# box for collision checks.
float64 length        # Body length, m
float64 width         # Body width, m
float64 height        # Body height, m
