inertia = { 
	{1.1, 0.1, 0.2},
	{0.3, 1.2, 0.4},
	{0.5, 0.6, 1.3},
}

pelvis = { mass = 9.3, com = { 1.1, 1.2, 1.3}, inertia = inertia }
thigh = { mass = 4.2, com = { 1.1, 1.2, 1.3}, inertia = inertia }
shank = { mass = 4.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }
foot = { mass = 1.1, com = { 1.1, 1.2, 1.3}, inertia = inertia }

bodies = {
	pelvis = pelvis,
	thigh_right = thigh,
	shank_right = shank,
	foot_right = foot,
	thigh_left = thigh,
	shank_left = shank,
	foot_left = foot,
}

joints = {
	freeflyer = {
		{ 0., 0., 0., 1., 0., 0.},
		{ 0., 0., 0., 0., 1., 0.},
		{ 0., 0., 0., 0., 0., 1.},
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.},
	},
	euler_zyx = {
		{ 0., 0., 1., 0., 0., 0.},
		{ 0., 1., 0., 0., 0., 0.},
		{ 1., 0., 0., 0., 0., 0.},
	},
	rotational_y = {
		{ 0., 1., 0., 0., 0., 0.},
	},
	fixed = {},
}

eye33 = {{1.0,0.0,0.,},
	 			 {0.0,1.0,0.,},
	 			 {0.0,0.0,1.,},}


return {
  --Named body fixed points. The names of the fields have been set for
  --historical reasons. It would make more sense if this was named instead
  -- local_points, with a field of 'r' rather than 'point'.
points = {
	{name = "Heel_Medial_L", 			body = "foot_right", point = {-0.080000, -0.042000, -0.091000,},},
	{name = "Heel_Lateral_L", 		body = "foot_right", point = {-0.080000, 0.042000, -0.091000,},},
	{name = "ForeFoot_Medial_L", 	body = "foot_right", point = {0.181788, -0.054000, -0.091000,},},
	{name = "ForeFoot_Lateral_L", body = "foot_right", point = {0.181788, 0.054000, -0.091000,},},
},
--Named local frames
local_frames = {
  {name = "Pocket_L",   body="thigh_left",   r={0.,  0.2, 0.}, E=eye33, },
  {name = "Pocket_R",   body="thigh_right",  r={0., -0.2, 0.}, E=eye33, },
},
--subject data
human_meta_data = {
  {age_group = "Young18To25",   
   age = 35.0,
   height = 1.73,
   weight = 81.68,
   gender = "male"},
},
--torque muscle models
--The name must contain a name of one of the MTGs  listed in JointTorqueSet : line 60 of addons/muscle/Millard2016TorqueMuscle.cc
millard2016_torque_muscles = {
  { name = "HipExtension_R",   								angle_sign = -1, torque_sign =  1, body ="thigh_right",  joint_index=1, act_time = 0.05, deact_time = 0.05,},
  { name = "HipFlexion_R",     								angle_sign = -1, torque_sign = -1, body ="thigh_right",  joint_index=1, act_time = 0.05, deact_time = 0.05,},
  { name = "KneeExtension_R",  								angle_sign =  1, torque_sign = -1, body ="shank_right",                 act_time = 0.05, deact_time = 0.05,},
  { name = "KneeFlexion_R",    								angle_sign =  1, torque_sign =  1, body ="shank_right",                 act_time = 0.05, deact_time = 0.05,},
  { name = "AnkleExtension_R", 								angle_sign = -1, torque_sign =  1, body ="foot_right" ,                 act_time = 0.05, deact_time = 0.05,},
  { name = "AnkleFlexion_R",   								angle_sign = -1, torque_sign = -1, body ="foot_right" ,                 act_time = 0.05, deact_time = 0.05,},
  { name = "AnkleFlexion_R_FpeHalfScale",     angle_sign = -1, torque_sign = -1, body ="foot_right" ,       act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.5,},
  { name = "AnkleFlexion_R_FisoHalfScale",    angle_sign = -1, torque_sign = -1, body ="foot_right" ,       act_time = 0.05, deact_time = 0.05, max_isometric_torque_scale = 0.5,},
  { name = "AnkleFlexion_R_OmegaHalfScale",   angle_sign = -1, torque_sign = -1, body ="foot_right" ,       act_time = 0.05, deact_time = 0.05, max_angular_velocity_scale = 0.5,},
  { name = "UnitExtensor_R",   								angle_sign = -1, torque_sign =  1, body ="thigh_right",  joint_index=2 },
  { name = "UnitFlexor_R",     								angle_sign = -1, torque_sign = -1, body ="thigh_right",  joint_index=2 },
  { name = "KneeExtension_R_Anderson2007",   angle_sign = -1, torque_sign = -1, body ="foot_right" ,       act_time = 0.05, deact_time = 0.05, max_angular_velocity_scale = 0.5,
    	data_set = "Anderson2007", age_group = "SeniorOver65", gender="female", active_torque_angle_scale = 2.0},  
},
frames = {
	{
		name = "pelvis",
		parent = "ROOT",
		body = bodies.pelvis,
		joint = joints.freeflyer,
		markers = {
			LASI = 	{ 0.047794, 0.200000, 0.070908,},
			RASI = 	{ 0.047794, -0.200000, 0.070908,},
			LPSI = 	{ -0.106106, 0.200000, 0.070908,},
			RPSI = 	{ -0.106106, -0.200000, 0.070908,},},		
	},
	{
		name = "thigh_right",
		parent = "pelvis",
		body = bodies.thigh_right,
		joint = joints.euler_zyx,
		markers = {
			RTHI = 	{ -0.007376, -0.000000, -0.243721,},
			RKNE = 	{ -0.011611, -0.000000, -0.454494,},},		
	},
	{
		name = "shank_right",
		parent = "thigh_right",
		body = bodies.thigh_right,
		joint = joints.rotational_y,	
	},
	{
		name = "foot_right",
		parent = "shank_right",
		body = bodies.foot_right,
		joint = joints.rotational_y,	
	},
	{
		name = "thigh_left",
		parent = "pelvis",
		body = bodies.thigh_left,
		joint = joints.euler_zyx,
	},
	{
		name = "shank_left",
		parent = "thigh_left",
		body = bodies.shank_left,
		joint = joints.rotational_y,
	},
	{
		name = "foot_left",
		parent = "shank_left",
		body = bodies.foot_left,
		joint = joints.rotational_y,
	},
},
}