return {
  gravity = {0., 0, -9.81},
    configuration = {
      axis_front  = { 1,  0, 0,},
      axis_right  = { 0, -1, 0,},
      axis_up     = { 0,  0, 1,},
  },
  points = {
    {name = "joint", body = "K1", point = {0., 1., 0.,},},
  },
  --1c. The revolute joint between K1 and K2 is made by adding 5 constraints between
  --these to bodies to constrain the ends to have the same position 
  --(introducing 3 constraints) and the same orientation along 2 axis.
  --
  --    Question: Each of these 5 constraints only differ in the 'axis' entry. 
  --          Look at each of the entries: which axis is not listed?
  --
  --1d. These kinematic constraints can be described at the position level
  --    and result in a set of DAEs. Since very few integrators can work with
  --    DAEs, these constraints are instead applied at the acceleration level
  --    by taking two derivatives of the constraint equations and working
  --    these constraints into the equations of motion using Lagrange 
  --    multipliers. Please see Sec. 3.4 of Featherstone & Orin for details.
  --
  --    These the resulting constraints can drift apart due to integration 
  --    error. To stablize the error, here we use a technique called 
  --    Baumgarte stablization. When 'baumgarte_enabled' is set to 'true'
  --    forces will be applied to the constraint in proportion to its constraint
  --    error at the postion and velocity levels. The scaling of this force
  --    is scaled by 1.0/stabilization_coefficient: smaller values of
  --    stabiliation_coefficient make the Baumgarte forces larger. For details
  --    on Baumgarte's method please read up to Sec. 3.1 of Bauchau & Laulusa.
  --
  --     Featherstone R, Orin D. Robot dynamics: equations and algorithms. 
  --     In Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference 
  --     on Robotics and Automation. Symposia Proceedings (Cat. No. 00CH37065) 2000 
  --     (Vol. 1, pp. 826-834). IEEE.
  --
  --     Bauchau OA, Laulusa A. Review of contemporary approaches for constraint
  --     enforcement in multibody systems. Journal of Computational and 
  --     Nonlinear Dynamics. 2008 Jan 1;3(1):011005.
  constraint_sets = {
    PinJoint = {
      {constraint_type = "loop", 
       name = "x",
       predecessor_body = "K1",
       successor_body = "K2",
       predecessor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.000,  1.0,  0.000,},
       },
       successor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.0, 0.0, 0.0,},
       },
       axis = {0, 0, 0, 1, 0, 0},
       stabilization_coefficient = 1.0,
       baumgarte_enabled = false,
      },
      {constraint_type = "loop", 
       name = "y",
       predecessor_body = "K1",
       successor_body = "K2",
       predecessor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.000,  1.0,  0.000,},
       },
       successor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.0, 0.0, 0.0,},
       },
       axis = {0, 0, 0, 0, 1, 0},
       stabilization_coefficient = 1.0,
       baumgarte_enabled = false,
      },
      {constraint_type = "loop", 
       name = "z",
       predecessor_body = "K1",
       successor_body = "K2",
       predecessor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.000,  1.0,  0.000,},
       },
       successor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.0, 0.0, 0.0,},
       },
       axis = {0, 0, 0, 0, 0, 1},
       stabilization_coefficient = 1.0,
       baumgarte_enabled = false,
      },
      {constraint_type = "loop", 
       name = "ry",
       predecessor_body = "K1",
       successor_body = "K2",
       predecessor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.000,  1.0,  0.000,},
       },
       successor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.0, 0.0, 0.0,},
       },
       axis = {0, 1, 0, 0, 0, 0},
       stabilization_coefficient = 1.0,
       baumgarte_enabled = false,
      },
      {constraint_type = "loop", 
       name = "rz",
       predecessor_body = "K1",
       successor_body = "K2",
       predecessor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.000,  1.0,  0.000,},
       },
       successor_transform = {
         E = {
           {1, 0, 0},
           {0, 1, 0},
           {0, 0, 1},
         },
         r = {0.0, 0.0, 0.0,},
       },
       axis = {0, 0, 1, 0, 0, 0},
       stabilization_coefficient = 1.0,
       baumgarte_enabled = false,
      },
    },
  },
  frames = {  
    --1a. Here K1 is attached to ground through a 1 dof revolute joint inertia
    --    in the x-axis direction
    {name  = "K1",
      parent  = "ROOT",
      visuals = {{ dimensions = { 0.05, 1.0, 0.05 },
                  color = { 1, 0, 0 },
                  mesh_center = { 0,  0.5, 0 },
                  src = "unit_cube.obj",
      },},
      joint   =
        {{ 1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,},},
      joint_frame = {
        r = {0., 0., 0.,},
      },
      body = {
        mass   = 1.0,
        com =   { 0.000000, 0.500000, 0.00000,},
        inertia = 
          {{ 8.333333333333333e-02 , 0.00, 0.00,},
          {  0.00, 8.333333333333333e-02 , 0.00,},
          {  0.00, 0.00, 8.333333333333333e-02 ,},},
      },
    },
    --1b. K2 is attached to ROOT through a 6 dof spatial joint.
    {name  = "K2",
      parent  = "ROOT",
      visuals = {{ dimensions = { 0.05, 1.0, 0.05 },
                  color = { 0, 0, 1 },
                  mesh_center = { 0,  0.5, 0 },
                  src = "unit_cube.obj",
      },},
      joint   =
        {{ 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000,},
         { 0.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000,},
         { 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 1.000000,},
         { 1.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000,},
         { 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 0.000000,},
         { 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000,},},
      joint_frame = {
        r = {0., 0., 0.,},
      },
      body = {
        mass   = 1.0,
        com =   { 0.000000, 0.500000, 0.00000,},
        inertia = 
          {{ 8.333333333333333e-02 , 0.00, 0.00,},
          {  0.00, 8.333333333333333e-02 , 0.00,},
          {  0.00, 0.00, 8.333333333333333e-02 ,},},
      },
    },
  },
}
