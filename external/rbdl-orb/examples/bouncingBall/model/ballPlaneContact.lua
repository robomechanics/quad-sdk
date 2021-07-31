model= {
  animation_settings = {
    force_scale = 0.1,
    torque_scale = 1,
    force_color = {1., 1., 0.},
    torque_color = {0., 1., 0.},
    force_transparency = 0.5,
    torque_transparency = 0.5,
  },
  identityMatrix = {
      {1.,0.,0.},
      {0.,1.,0.},
      {0.,0.,1.},       
  },
  gravity = {0., 0., -9.81},
  configuration = {
    axis_front  = { 1, 0, 0,},
    axis_right  = { 0, -1, 0,},
    axis_up     = { 0, 0, 1,},
  },
  frames = {    
    {
      name    = "Ball",
      parent  = "ROOT",
      visuals = {{
          dimensions = {1.,1.,1.},
          color       = { 1, 0, 0 },
          mesh_center = { 0, 0.0, 0 },
          src         = "unit_sphere_medres.obj",
      },},
      body    = {   
        mass    = 1., 
        com     = {0., 0, 0.}, 
        inertia = {
          {0.1,0.,0.},
          {0.,0.1,0.},
          {0.,0.,0.1}, 
        },
      },
      joint   = {
        { 0., 0., 0., 1., 0., 0.},
        { 0., 0., 0., 0., 0., 1.},
        { 0., 1., 0., 0., 0., 0.},
      },
      joint_frame = {
          r = {0., 0., 0.},
      }
    },
  },
};

--dofile("generateEnumHeader.lua");

return model;
