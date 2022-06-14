# NMPC Controller

## Overview

This package implements a nonlinear model predictive controller (NMPC) for agile quadrupedal navigation. The package produces ground reaction force (GRF) and body state plans which guide the robot from its current state to track the desired trajectory according to the contact schedule. The primary algorithm is collocation trajectory planning constructed by [CasADi] supporting multiple dynamics model (default is a centroidal dynamics one) and is solved by [IPOPT] through a customed C++ interface to produce GRF plans considering contact schedule.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Yanhao Yang<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainers: Yanhao Yang (yanhaoy@andrew.cmu.edu), Joe Norby (jnorby@andrew.cmu.edu), and Jiming Ren (jimingre@andrew.cmu.edu)**

The NMPC Controller package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

If you use this work in an academic context, please cite the following publication(s):

* J. Norby, Y. Yang, A. Tajbakhsh et al., “Quad-SDK,” 2022. \[Online\]. Available: https://github.com/robomechanics/quad-software ([paper])

         @misc{norby2022quad, 
         title={{Quad-SDK}}, 
         url={https://github.com/robomechanics/quad-software}, 
         author={Norby, Joseph and Yang, Yanhao and Tajbakhsh, Ardalan and Ren, Jiming and Yim, Justin K. and Stutt, Alexandra and Yu, Qishun and Johnson, Aaron M.},
         year={2022}} 

### Unit Tests

Run the unit tests with

	catkin run_tests nmpc_controller

## Usage

It is a C++ class that can be included in other ROS packages.

## Config files

* **nmpc_controller.yaml** Sets NMPC hyperparameters.

#### Parameters

* **`panic_weights`** (double, default: 200.0)

	The linear panalty of the state panic variables.

* **`constraint_panic_weights`** (double, default: 20.0)

	The linear panalty of the constraint panic variables.

* **`Q_temporal_factor`** (double, default: 10.0)

	The temporal factor of the state cost weight.

* **`R_temporal_factor`** (double, default: 1.0)

	The temporal factor of the control cost weight.

* **`friction_coefficient`** (double, default: 0.3)

	The friction coefficient for GRF.

* **`enable_variable_horizon`** (bool, default: false)

	Turn on varialbe horzion or not.

* **`min_horizon_length`** (int, default: 10)

	Minimum horizon length when it's varaible.

* **`enable_mixed_complexity`** (bool, default: false)

	Enable mixed complexity or not

* **`enable_adaptive_complexity`** (bool, default: false)

	Enable adaptive complexity or not.

* **`fixed_complex_idxs`** (vector, default: [])

	Fixed complex finite element index.

* **`fixed_complex_head`** (int, default: 0)

	Fixed complex finite element number at the head.

* **`fixed_complex_tail`** (int, default: 0)

	Fixed complex finite element number at the tail.

#### Parameters for each type of model

* **`x_dim`** (int, default: 12)

	State dimension.

* **`u_dim`** (int, default: 12)

	Control dimension.

* **`g_dim`** (int, default: 28)

	Constraint dimension.

* **`x_weights`** (vector, default: [5.0, 5.0, 5.0, 0.5, 0.5, 0.5, 0.1, 0.1, 0.2, 0.05, 0.05, 0.01])

	State cost weights.

* **`u_weights`** (vector, default: [!!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5, !!float 5e-5])

	Control cost weights.

* **`x_lb`** (vector, default: [!!float -2e19, !!float -2e19, 0, -3.14159, -3.14159, -3.14159, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19])

	State lower bound.

* **`x_ub`** (vector, default: [!!float 2e19, !!float 2e19, !!float 2e19, 3.14159, 3.14159, 3.14159, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19])

	State upper bound.

* **`x_lb_soft`** (vector, default: [!!float -2e19, !!float -2e19, 0, -3.14159, -3.14159, -3.14159, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19])

	Soft state lower bound.

* **`x_ub_soft`** (vector, default: [!!float 2e19, !!float 2e19, !!float 2e19, 3.14159, 3.14159, 3.14159, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19, !!float 2e19])

	Soft state upper bound.

* **`u_lb`** (vector, default: [!!float -2e19, !!float -2e19, 10, !!float -2e19, !!float -2e19, 10, !!float -2e19, !!float -2e19, 10, !!float -2e19, !!float -2e19, 10])

	Control lower bound.

* **`u_ub`** (vector, default: [!!float 2e19, !!float 2e19, 250, !!float 2e19, !!float 2e19, 250, !!float 2e19, !!float 2e19, 250, !!float 2e19, !!float 2e19, 250])

	Control upper bound.

* **`g_lb`** (vector, default: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19, !!float -2e19])

	Constraint lower bound.

* **`g_ub`** (vector, default: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

	Constraint upper bound.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).


[paper]: https://leggedrobots.org/assets/pdfs/paper22.pdf
[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[CasADi]: https://web.casadi.org/
[IPOPT]: https://coin-or.github.io/Ipopt/
