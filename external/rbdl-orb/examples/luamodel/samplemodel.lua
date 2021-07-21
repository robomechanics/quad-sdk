--[[
--  This is an example model for the RBDL addon luamodel. You need to
--  enable RBDL_BUILD_ADDON_LUAMODEL to be able to use this file.
--]]

print ("This is some output from the model file")

inertia = { 
	{1.1, 0.1, 0.2},
	{0.3, 1.2, 0.4},
	{0.5, 0.6, 1.3}
}

pelvis = { mass = 9.3, com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh = { mass = 4.2, com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank = { mass = 4.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }
foot = { mass = 1.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }

bodies = {
	pelvis = pelvis,
	thigh_right = thigh,
	shank_right = shank,
	thigh_left = thigh,
	shank_left = shank
}

joints = {
	freeflyer = {
		{ 0., 0., 0., 1., 0., 0.},
		{ 0., 0., 0., 0., 1., 0.},
		{ 0., 0., 0., 0., 0., 1.},
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
	spherical_zyx = {
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.}
	},
	rotational_y = {
		{ 0., 1., 0., 0., 0., 0.}
	},
	fixed = {}
}

model = {
	gravity = {0., 0., -9.81},

	frames = {
		{
			name = "pelvis",
			parent = "ROOT",
			body = bodies.pelvis,
			joint = joints.freeflyer,
		},
		{
			name = "thigh_right",
			parent = "pelvis",
			body = bodies.thigh_right,
			joint = joints.spherical_zyx,
			joint_frame = {
				r = {0.0, -0.15, 0.},
				E = {
					{1., 0., 0.},
					{0., 1., 0.},
					{0., 0., 0.}
				}
			}
		},
		{
			name = "shank_right",
			parent = "thigh_right",
			body = bodies.thigh_right,
			joint = joints.rotational_y,
			joint_frame = {
				r = {0.0, 0., -0.42},
			},
		},
		{
			name = "foot_right",
			parent = "shank_right",
			body = bodies.thigh_right,
			joint_frame = {
				r = {0.0, 0., -0.41}
			},
		},
		{
			name = "thigh_left",
			parent = "pelvis",
			body = bodies.thigh_left,
			joint = joints.spherical_zyx,
			joint_frame = {
				r = {0.0, 0.15, 0.},
				E = {
					{1., 0., 0.},
					{0., 1., 0.},
					{0., 0., 0.}
				}
			}
		},
		{
			name = "shank_left",
			parent = "thigh_left",
			body = bodies.thigh_left,
			joint = joints.rotational_y,
			joint_frame = {
				r = {0.0, 0., -0.42},
			},
		},
		{
			name = "foot_left",
			parent = "shank_left",
			body = bodies.thigh_left,
			joint_frame = {
				r = {0.0, 0., -0.41},
			},
		},
	}
}

return model
