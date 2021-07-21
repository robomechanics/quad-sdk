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
  {name = "Heel_Right",     body="foot_right",  r={-0.1,   0.0,  -0.15}, E=eye33, },
  {name = "ForeFoot_Right", body="foot_right",  r={ 0.3,   0.0,  -0.15}, E=eye33, },
  {name = "GroundFrame",    body="ROOT",        r={0.,0.,0.}, E=eye33,},
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
  { name = "HipExtension_R",   angle_sign = -1, torque_sign =  1, body ="thigh_right",  joint_index=1, act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
  { name = "HipFlexion_R",     angle_sign = -1, torque_sign = -1, body ="thigh_right",  joint_index=1, act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
  { name = "KneeExtension_R",  angle_sign =  1, torque_sign = -1, body ="shank_right",                 act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
  { name = "KneeFlexion_R",    angle_sign =  1, torque_sign =  1, body ="shank_right",                 act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
  { name = "AnkleExtension_R", angle_sign = -1, torque_sign =  1, body ="foot_right" ,                 act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
  { name = "AnkleFlexion_R",   angle_sign = -1, torque_sign = -1, body ="foot_right" ,                 act_time = 0.05, deact_time = 0.05, passive_element_torque_scale = 0.0,},
},
constraint_set_phases = {
  "left_foot",
  "right_foot",
  "right_foot",
  "left_foot",  
},
constraint_sets = {
	left_foot = {
		{	constraint_type = 'contact', name = 'leftHeelMedialXYZ', id = 1,        
    	point_name = 'Heel_Medial_L', normal_sets = {	{1.,0.,0.,},
    																								{0.,1.,0.,},
    																								{0.,0.,1.,} },
  	},
		{	constraint_type = 'contact', name = 'leftForeMedialYZ', id = 1,        
    	point_name = 'ForeFoot_Medial_L', normal_sets = {{1.,0.,0.,},{0.,0.,1.}},
  	},  
		{	constraint_type = 'contact', name = 'leftForeLateralZ', id = 1,        
    	point_name = 'ForeFoot_Lateral_L', normal = {0.,0.,1.},
  	},    		
	},
	right_foot = {
		 {  constraint_type = 'loop', name = 'loopRightHeel', id = 2, 
        predecessor_local_frame = 'Heel_Right',
        successor_local_frame = 'GroundFrame',
        axis_sets =  {{1., 0., 0., 0., 0., 0.},
                      {0., 1., 0., 0., 0., 0.},
                      {0., 0., 1., 0., 0., 0.},
                      {0., 0., 0., 1., 0., 0.},
                      {0., 0., 0., 0., 1., 0.},
                      {0., 0., 0., 0., 0., 1.},},
        stabilization_coefficient = 0.1,
        enable_stabilization = true,
      },
	},
},
frames = {
	{
		name = "pelvis",
		parent = "ROOT",
		body = bodies.pelvis,
		joint = {"JointTypeFloatingBase"},
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
		joint_frame = {
		    r =   { 0.000000, -0.100000, 0.000000,},
		    E = eye33,
		},		
		joint = {"JointTypeEulerYXZ"},
		markers = {
			RTHI = 	{ -0.007376, -0.000000, -0.243721,},
			RKNE = 	{ -0.011611, -0.000000, -0.454494,},},		
	},
	{
		name = "shank_right",
		parent = "thigh_right",
		body = bodies.thigh_right,
		joint_frame = {
		    r =   { 0.000000, 0.00000, -0.500000,},
		    E = eye33,
		},
		joint = {{0.,1.,0.,0.,0.,0.},},	
	},
	{
		name = "foot_right",
		parent = "shank_right",
		body = bodies.foot_right,
		joint_frame = {
		    r =   { 0.000000, 0.00000, -0.500000,},
		    E = eye33,
		},		
		joint = {{0.,1.,0.,0.,0.,0.},
	           {1.,0.,0.,0.,0.,0.},},		
	},
	{
		name = "thigh_left",
		parent = "pelvis",
		body = bodies.thigh_left,
		joint_frame = {
		    r =   { 0.000000, 0.10000, 0.000000,},
		    E = eye33,
		},				
		joint = {"JointTypeEulerYXZ"},
	},
	{
		name = "shank_left",
		parent = "thigh_left",
		body = bodies.shank_left,
		joint_frame = {
		    r =   { 0.000000, 0.00000, -0.500000,},
		    E = eye33,
		},		
		joint = {{0.,1.,0.,0.,0.,0.},},
	},
	{
		name = "foot_left",
		parent = "shank_left",
		body = bodies.foot_left,
		joint_frame = {
		    r =   { 0.000000, 0.00000, -0.500000,},
		    E = eye33,
		},				
		joint = {{0.,1.,0.,0.,0.,0.},
	           {1.,0.,0.,0.,0.,0.},},	
	},
},
}