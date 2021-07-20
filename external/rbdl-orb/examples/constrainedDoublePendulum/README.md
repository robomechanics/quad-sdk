constrainedPendulumForwardDynamics
Copyright (c) 2019 Matthew Millard <matthew.millard@iwr.uni-heidelberg.de>
Licensed under the zlib license. See LICENSE for more details.

# Quick Start

This example reads in a double pendulum in which the first revolute joint is 
described using joint coordinates, and the second revolute joint is made 
using 5 constraints between the ends of the two pendulums. The model is
read into the c++ program, rbdl is used to compute its state derivative, and
finally Boost is used to numerically integrate its movements. To get started

1. Make a 'build' folder here

2. Run cmake from the 'build' folder. You will be asked to set CUSTOM_RBDL_PATH,
   which is the path to the install folder of the version of RBDL that you 
   would like to use. 

    If you have a system-wide installation of RBDL this folder is '/usr/local'

3. From a terminal in the build directory run
    ```
    make
    ```

    *Note*: Currently there are a lot of warnings that originate, I think, from
           the new code that has been added in luamodel to read in a model
           with constraints. While this code functions, it will be getting
           an overhaul in the coming months.

4. From a terminal in the build directory run
    ```
    ./constrainedDoublePendulumForwardDynamics
    ```
5. If everything works you should see a bunch of data printed to the screen and
   2 files written to the output folder. See the 'Example Output' section for
  an example.

6. To view an animation, from a terminal in the 'constrainedDoublePendulum' 
  folder run
    ```
    meshup model/constrainedDoublePendulum.lua output/meshup.csv
    ```

# Detailed Code Tour

  Time commitment: 30-45 minutes


This simple example reads in a double pendulum model. To begin, open
model/constrainedDoublePendulum.lua in a text editor. As before the comments
below appear in the constrainedDoublePendulum file:

1a. Here K1 is attached to ROOT through a 1 dof revolute joint inertia
in the x-axis direction

1b. K2 is attached to ROOT through a 6 dof spatial joint
  <strong>Question</strong>: How many degrees of freedom does this system have?
  <strong>Homework</strong>: The specific form of this 6 dof joint uses Euler angles which have 
            has a nasty problem that occurs at certain angles called 
            'gimbal lock'. If you have never heard of gimbal lock, go and read
            this Wikipedia entry: https://en.wikipedia.org/wiki/Gimbal_lock

1c. The revolute joint between K1 and K2 is made by adding 5 constraints between
    these to bodies to constrain the ends to have the same position 
    (introducing 3 constraints) and the same orientation along 2 axis.

  <strong>Question</strong>: 
  Each of these 5 constraints only differ in the 'axis' entry. 
  Look at each of the entries: which axis is not listed?

  <strong>Question</strong>: 
  Look closely at the entries with words 'predecessor' and 'successor'
  in them and answer the following: What is the local transform for the point 
  on K1 to which the constraints are applied? What is the local transform for 
  the point on K2 to which the constraints are applied?

*Note*: This very verbose loop-constraint description is going to be 
      replaced with a much more compact version in the coming months. 

1d. These kinematic constraints can be described at the position level
    and result in a set of DAEs. Since very few integrators can work with
    DAEs, these constraints are instead applied at the acceleration level
    by taking two derivatives of the constraint equations and working
    these constraints into the equations of motion using Lagrange 
    multipliers. Please see Sec. 3.4 of Featherstone & Orin for details.

  These the resulting constraints can drift apart due to integration 
  error. To stablize the error, here we use a technique called 
  Baumgarte stablization. When 'baumgarte_enabled' is set to 'true'
  forces will be applied to the constraint in proportion to its constraint
  error at the postion and velocity levels. The scaling of this force
  is scaled by 1.0/stabilization_coefficient: smaller values of
  stabiliation_coefficient make the Baumgarte forces larger. For details
  on Baumgarte's method please read up to Sec. 3.1 of Bauchau & Laulusa.

     Featherstone R, Orin D. Robot dynamics: equations and algorithms. 
     In Proceedings 2000 ICRA. Millennium Conference. IEEE International Conference 
     on Robotics and Automation. Symposia Proceedings (Cat. No. 00CH37065) 2000 
     (Vol. 1, pp. 826-834). IEEE.

     Bauchau OA, Laulusa A. Review of contemporary approaches for constraint
     enforcement in multibody systems. Journal of Computational and 
     Nonlinear Dynamics. 2008 Jan 1;3(1):011005.

*Note*: The Baumgarte stabilization coefficient is going to be made more intuitive
      in the coming months. 

2. Open src/constrainedDoublePendulumForwardDynamics.cc
   The vast majority of this file follows the same layout as does the 
   pendulumForwardDynamics.cc file. The only significant differences are 
   the following:

2a. Some extra work is needed to load in the model, and the 
    constraint sets

2c. A special forward dynamics function needs to be called in
    order to compute qdd which simultaneously satisfies the 
    constraint set and the equations of motion.

2d. The position-level and velocity level constraint set errors
    can be fished out of the vectors in the ConstraintSet struct

  *Note*: In the coming months there will be some functions added so that you
  can easily access this information.

  <strong>Homework</strong>: After reading Featherstone & Orin Sec. 3.4, go and have a look
  at the fields in the ConstraintSet struct in 
  rbdl-orb/include/rbdl/Constraints.h

  At the same time, open the doxygen for RBDL and read the sections
  on Constraints and ConstraintSets

2e. If you're interested, after simulating this system you can plot the
    data in output/simulationData.csv to observe the Euclidean norm of the
    constraint error at the position and velocity levels which appear in the
    output/simulationData.csv file. 

  <strong>Question</strong>:
  If you re-run the simulation with looser integration tolerances what 
  happens to the integration errors? (Be sure to save the old 
  simulationData.csv file before it is over written).

  <strong>Question</strong>:
  Now, if you enable Baumgarte stabiliation how does the error change?


# Example Output

DoF: 7
==============================
1. Forward Dynamics 
==============================
Columns
      t,        ke,       pe,    ke+pe ,norm(cons_pos), norm(cons_vel)
0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000
0.030000, 0.074239, -0.074239, 0.000000, 0.000000, 0.000000
0.060000, 0.296914, -0.296914, 0.000000, 0.000000, 0.000000
0.090000, 0.667669, -0.667669, 0.000000, 0.000000, 0.000000
0.120000, 1.185130, -1.185130, 0.000000, 0.000000, 0.000000
0.150000, 1.845890, -1.845890, -0.000000, 0.000000, 0.000000
0.180000, 2.643470, -2.643470, -0.000000, 0.000000, 0.000000
0.210000, 3.567814, -3.567814, -0.000000, 0.000000, 0.000000
0.240000, 4.605609, -4.605608, 0.000000, 0.000000, 0.000000
0.270000, 5.741238, -5.741238, 0.000000, 0.000000, 0.000000
0.300000, 6.957738, -6.957738, 0.000000, 0.000000, 0.000000
0.330000, 8.237324, -8.237324, -0.000000, 0.000000, 0.000000
0.360000, 9.561525, -9.561525, -0.000000, 0.000000, 0.000000
0.390000, 10.911164, -10.911164, -0.000000, 0.000000, 0.000000
0.420000, 12.266282, -12.266282, -0.000000, 0.000000, 0.000000
0.450000, 13.605772, -13.605772, -0.000000, 0.000000, 0.000000
0.480000, 14.905985, -14.905985, -0.000000, 0.000000, 0.000000
0.510000, 16.136976, -16.136976, 0.000000, 0.000000, 0.000000
0.540000, 17.254754, -17.254753, 0.000001, 0.000000, 0.000000
0.570000, 18.190859, -18.190859, 0.000000, 0.000000, 0.000000
0.600000, 18.857995, -18.857993, 0.000002, 0.000000, 0.000000
0.630000, 19.207403, -19.207401, 0.000002, 0.000000, 0.000000
0.660000, 19.277718, -19.277717, 0.000002, 0.000000, 0.000000
0.690000, 19.139624, -19.139622, 0.000002, 0.000000, 0.000000
0.720000, 18.838033, -18.838030, 0.000003, 0.000000, 0.000000
0.750000, 18.388297, -18.388294, 0.000003, 0.000000, 0.000000
0.780000, 17.788048, -17.788045, 0.000003, 0.000000, 0.000000
0.810000, 17.025660, -17.025657, 0.000003, 0.000000, 0.000000
0.840000, 16.084710, -16.084708, 0.000002, 0.000000, 0.000000
0.870000, 14.949799, -14.949797, 0.000002, 0.000000, 0.000000
0.900000, 13.622800, -13.622797, 0.000003, 0.000000, 0.000000
0.930000, 12.147238, -12.147234, 0.000004, 0.000000, 0.000000
0.960000, 10.606298, -10.606295, 0.000003, 0.000000, 0.000000
0.990000, 9.084459, -9.084456, 0.000003, 0.000000, 0.000000
1.020000, 7.641065, -7.641062, 0.000003, 0.000000, 0.000000
1.050000, 6.311035, -6.311032, 0.000003, 0.000000, 0.000000
1.080000, 5.113535, -5.113532, 0.000003, 0.000000, 0.000000
1.110000, 4.058634, -4.058631, 0.000003, 0.000000, 0.000000
1.140000, 3.151293, -3.151290, 0.000003, 0.000000, 0.000000
1.170000, 2.393708, -2.393705, 0.000003, 0.000000, 0.000000
1.200000, 1.786738, -1.786735, 0.000003, 0.000000, 0.000000
1.230000, 1.330730, -1.330727, 0.000003, 0.000000, 0.000000
1.260000, 1.025803, -1.025800, 0.000003, 0.000000, 0.000000
1.290000, 0.871439, -0.871436, 0.000003, 0.000000, 0.000000
1.320000, 0.865295, -0.865292, 0.000003, 0.000000, 0.000000
1.350000, 1.001860, -1.001857, 0.000003, 0.000000, 0.000000
1.380000, 1.272625, -1.272622, 0.000003, 0.000000, 0.000000
1.410000, 1.668330, -1.668327, 0.000003, 0.000000, 0.000000
1.440000, 2.181119, -2.181116, 0.000003, 0.000000, 0.000000
1.470000, 2.804822, -2.804819, 0.000003, 0.000000, 0.000000
1.500000, 3.533948, -3.533944, 0.000003, 0.000000, 0.000000
1.530000, 4.362465, -4.362462, 0.000003, 0.000000, 0.000000
1.560000, 5.282786, -5.282783, 0.000003, 0.000000, 0.000000
1.590000, 6.284982, -6.284979, 0.000003, 0.000000, 0.000000
1.620000, 7.356270, -7.356267, 0.000003, 0.000000, 0.000000
1.650000, 8.480957, -8.480954, 0.000003, 0.000000, 0.000000
1.680000, 9.641140, -9.641137, 0.000003, 0.000000, 0.000000
1.710000, 10.818474, -10.818471, 0.000003, 0.000000, 0.000000
1.740000, 11.996927, -11.996924, 0.000003, 0.000000, 0.000000
1.770000, 13.165750, -13.165747, 0.000003, 0.000000, 0.000000
1.800000, 14.321085, -14.321082, 0.000003, 0.000000, 0.000000
1.830000, 15.464395, -15.464392, 0.000003, 0.000000, 0.000000
1.860000, 16.596183, -16.596180, 0.000003, 0.000000, 0.000000
1.890000, 17.702499, -17.702494, 0.000005, 0.000000, 0.000000
1.920000, 18.723762, -18.723747, 0.000015, 0.000000, 0.000001
1.950000, 19.468539, -19.468529, 0.000010, 0.000000, 0.000001
1.980000, 19.539562, -19.539539, 0.000022, 0.000000, 0.000001
2.010000, 18.905439, -18.905418, 0.000022, 0.000000, 0.000001
2.040000, 17.967975, -17.967946, 0.000029, 0.000000, 0.000000
2.070000, 16.937479, -16.937448, 0.000031, 0.000001, 0.000001
2.100000, 15.878826, -15.878795, 0.000031, 0.000001, 0.000001
2.130000, 14.806040, -14.806009, 0.000031, 0.000001, 0.000001
2.160000, 13.716756, -13.716724, 0.000031, 0.000001, 0.000000
2.190000, 12.607048, -12.607016, 0.000032, 0.000001, 0.000000
2.220000, 11.478540, -11.478508, 0.000032, 0.000001, 0.000000
2.250000, 10.340412, -10.340380, 0.000032, 0.000001, 0.000000
2.280000, 9.207819, -9.207787, 0.000032, 0.000001, 0.000000
2.310000, 8.098674, -8.098643, 0.000032, 0.000001, 0.000000
2.340000, 7.030552, -7.030521, 0.000032, 0.000001, 0.000000
2.370000, 6.018643, -6.018611, 0.000032, 0.000001, 0.000000
2.400000, 5.074870, -5.074838, 0.000032, 0.000001, 0.000000
2.430000, 4.207921, -4.207888, 0.000032, 0.000001, 0.000000
2.460000, 3.424007, -3.423975, 0.000032, 0.000001, 0.000000
2.490000, 2.728595, -2.728562, 0.000032, 0.000001, 0.000001
2.520000, 2.129654, -2.129621, 0.000033, 0.000001, 0.000001
2.550000, 1.641499, -1.641466, 0.000033, 0.000001, 0.000001
2.580000, 1.283740, -1.283707, 0.000033, 0.000001, 0.000001
2.610000, 1.072645, -1.072611, 0.000033, 0.000001, 0.000000
2.640000, 1.015398, -1.015365, 0.000033, 0.000001, 0.000000
2.670000, 1.112701, -1.112667, 0.000033, 0.000001, 0.000000
2.700000, 1.363109, -1.363076, 0.000033, 0.000001, 0.000000
2.730000, 1.765295, -1.765261, 0.000033, 0.000001, 0.000000
2.760000, 2.318670, -2.318636, 0.000033, 0.000001, 0.000000
2.790000, 3.023257, -3.023224, 0.000033, 0.000001, 0.000000
2.820000, 3.879209, -3.879176, 0.000033, 0.000001, 0.000000
2.850000, 4.886072, -4.886039, 0.000033, 0.000001, 0.000000
2.880000, 6.041792, -6.041758, 0.000033, 0.000001, 0.000000
2.910000, 7.341343, -7.341309, 0.000034, 0.000001, 0.000000
2.940000, 8.774690, -8.774656, 0.000034, 0.000001, 0.000000
2.970000, 10.323193, -10.323159, 0.000034, 0.000001, 0.000000
3.000000, 11.951996, -11.951961, 0.000035, 0.000001, 0.000000
Columns
      t,        ke,       pe,    ke+pe ,norm(cons_pos), norm(cons_vel)
Wrote: ../output/meshup.csv (meshup animation file)
Wrote: ../output/simulationData.csv (error data)
