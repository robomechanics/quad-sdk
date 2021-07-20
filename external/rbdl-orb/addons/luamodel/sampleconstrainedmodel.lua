-- 5b3d.lua
-- Copyright (c) 2016 Davide Corradi <davide.corradi@iwr.uni-heidelberg.de>

-- Parameters

m1 = 2
l1 = 2
r1 = 0.2
Izz1 = m1 * l1 * l1 / 3 

m2 = 2
l2 = 2
r2 = 0.2
Izz2 = m2 * l2 * l2 / 3

bodies = {

  virtual = {
    mass = 0,
    com = {0, 0, 0},
    inertia = {
      {0, 0, 0},
      {0, 0, 0},
      {0, 0, 0},
    },
  },

  link1 = {
    mass = m1,
    com = {l1/2, 0, 0},
    inertia = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, Izz1},
    },
  },

  link2 = {
    mass = m2,
    com = {l2/2, 0, 0},
    inertia = {
      {1, 0, 0},
      {0, 1, 0},
      {0, 0, Izz2},
    },
  },

}

joints = {

  revZ = {
    {0, 0, 1, 0, 0, 0},
  },

  trnXYZ = {
    {0, 0, 0, 1, 0, 0},
    {0, 0, 0, 0, 1, 0},
    {0, 0, 0, 0, 0, 1},
  },

}

meshes = {
  link1 = {
    name = 'link1',
    dimensions = {l1, r1, r1},
    color = {1, 0, 0},
    src = 'unit_cylinder_medres_z.obj',
    mesh_center = {l1/2, 0, 0},
  },
  link2 = {
    name = 'link2',
    dimensions = {l2, r2, r2},
    color = {0, 1, 0},
    src = 'unit_cylinder_medres_z.obj',
    mesh_center = {l2/2, 0, 0},
  },
}

eye33 = {
  {1.,0.,0.,},
  {0.,1.,0.,},
  {0.,0.,1.,},
}

model = {

  gravity = {0, 0, 0},

   configuration = {
    axis_front = { 1.,  0.,  0.},
    axis_right = { 0., -1.,  0.},
    axis_up    = { 0.,  0.,  1.},
  },

  frames = {
    
    {
      name = 'base',
      parent = 'ROOT',
      body = bodies.virtual,
      joint = joints.trnXYZ,
    },
    {
      name = 'l11',
      parent = 'base',
      body = bodies.link1,
      joint = joints.revZ,
      visuals = { meshes.link1 },
    },
    {
      name = 'l12',
      parent = 'l11',
      body = bodies.link2,
      joint = joints.revZ,
      joint_frame = {
        r = {l1, 0, 0},
      },
      visuals = { meshes.link2 },
    },
    {
      name = 'l21',
      parent = 'base',
      body = bodies.link1,
      joint = joints.revZ,
      visuals = { meshes.link1 },
    },
    {
      name = 'l22',
      parent = 'l21',
      body = bodies.link2,
      joint = joints.revZ,
      joint_frame = {
        r = {l1, 0, 0},
      },
      visuals = { meshes.link2 },
    },

  },

  --Named body fixed points. The names of the fields have been set for
  --historical reasons. It would make more sense if this was named instead
  -- local_points, with a field of 'r' rather than 'point'.
  points = {
    {name = 'base_origin', body='base', point={0.,0.,0.},},
  },

  --Named local frames
  local_frames = {
    {name = 'l12_A',       body='l12',  r={l2,0.,0.}, E=eye33,},
    {name = 'l22_B',       body='l22',  r={0.,0.,0.}, E=eye33,},
  },
  constraint_set_phases = {
    "individual_constraints",
    "constraints_in_sets",
    "constraints_in_sets",
    "individual_constraints",  
  },
  constraint_sets = {
    individual_constraints = {
      { -- Contact Constraint: verbose syntax
        -- Note: Contact Constraints do not have the 'enable_stablization' flag
        --       exposed (it is currently ignored if present) because this 
        --       type of constraint is very well numerically behaved and rarely
        --       suffers from drift.
        constraint_type = 'contact', name = 'contactBaseX', id = 2,
        body = 'base', point = {0, 0, 0}, normal = {1., 0., 0.},
      }, 
      { -- Contact Constraint: contact syntax - makes use of named points
        -- and passes a set of normals
        constraint_type = 'contact', name = 'contactBaseYZ', id = 3,        
        point_name = 'base_origin', normal_sets = {{0.,1.,0.,},{0.,0.,1.}},
      }, 
      { -- Loop Constraint: compact syntax - makes use of named local_frames        
        constraint_type = 'loop', name = 'loopL12L22Tx', id = 1, 
        predecessor_local_frame = 'l12_A',
        successor_local_frame = 'l22_B',
        axis =  {0., 0., 0., 1., 0., 0.},
        stabilization_coefficient = 0.1,
      },
      { -- Loop Constraint: verbose syntax.
        constraint_type = 'loop', name = 'loopL12L22Ty', id = 2,
        predecessor_body = 'l12',
        predecessor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {l2, 0, 0},
        },
        successor_body = 'l22',
        successor_transform = {
          E = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1},
          },
          r = {0, 0, 0},
        },
        axis = {0., 0., 0., 0., 1., 0.},      
        stabilization_coefficient = 0.1,
        enable_stabilization = true,
      },      
    },
    constraints_in_sets = {
      { -- Contact Constraint: compact syntax - use of named points and 
        -- normal sets
        constraint_type = 'contact',name = 'contactBaseXYZ',id = 2,
        point_name = 'base_origin',
        normal_sets = {{1., 0., 0.},
                       {0., 1., 0.},
                       {0., 0., 1.},},        
      },
      { -- Loop Constraint: compact syntax - use of named local_frames 
        -- and normal sets
        constraint_type = 'loop', name = 'loopL12L22TxTy', id = 1,
        predecessor_local_frame = 'l12_A',
        successor_local_frame = 'l22_B',              
        axis_sets = {{0., 0., 0., 1., 0., 0.},
                     {0., 0., 0., 0., 1., 0.},},
        stabilization_coefficient = 0.1,        
        enable_stabilization = false,
      },
    },
  },  

}

return model