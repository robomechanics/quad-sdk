# Python wrapper for RBDL

This wrapper uses Cython to wrap RBDL in a Python module. The code requires
C++11 features and must be compiled with the flags ```-std=c++11``` (or on
older compilers: ```-std=c++0x```). It closely follows the C++ API. All
functions are found in the module ```rbdl```, e.g.

    rbdl.ForwardDynamics (model, q, qdot, tau, qddot)

computes the accelerations qddot of the forward dynamics for given model,
q, qdot, tau. 

Arguments are all passed as reference and where possible, the wrapper
avoids copying values, especially for larger matrices such as Jacobians,
and joint-space inertia matrix.

All functions have embedded signatures to ease the use from within IPython,
e.g. ```rbdl.ForwardDynamics?``` shows required arguments for the function.

# Highlights

Wrappers for the following features are already implemented:

* supports model creation from Python code
* supports model loading from LuaModel and URDF
* operates directly on raw numpy data for Jacobians and joint-space inertia
	matrix - no copying required!
* direct access to almost all values of the Model structure
* all functions of Dynamics.h are wrapped:
  - O(n) inverse dynamics via RNEA
	- O(n) forward dynamics via ABA
	- Coriolis term computation via simplified RNEA
	- computation of joint-space inertia matrix via CRBA
* kinematic computations:
  - body <-> world transformations
	- body point positions, velocities, and accelerations, including their
		6-D counterparts
	- point Jacobians (translations)
	- 6-D Jacobians (angular and linear movement)
	- Spatial 6-D body Jacobians
* model mass, Center of Mass (CoM), CoM velocity, centroidal angular momentum

# Differences to the C++ API

The wrapper function ```rbdl.CalcCenterOfMass``` has a scalar return value
which is the mass of the model. Therefore the function does not use the
mass parameter when calling it.

# ToDo

* wrapping of constraint sets, and contact dynamics
* inverse kinematics
* documentation
