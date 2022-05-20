# Local Planner

## Overview

This package determines the contact timing, locations, and forces, to execute a given robot body plan. Local body and footstep plans are computed separately, and heuristically combined later on. 

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Joe Norby<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainers: Yanhao Yang (yanhaoy@andrew.cmu.edu) and Alex Stutt (astutt@andrew.cmu.edu)**

The Local Planner package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Usage

## Config files

* **local_planner.yaml** Sets planning hyperparameters, for local body and footstep plans.

## Nodes

### local_planner

Calculates a local robot body plan, and discrete and continuous local footstep plans. 

Local Body Planer
* Uses NMPC to find ground reaction forces that best track a nominal body trajectory. 

Local Footstep Planner
* Uses the a given local body plan to calculate foot trajectories. It computes a contact schedule, and then selects discrete foothold positions using dynamic and kinematic heuristics similar to Raibert's heuristic. 

#### Subscribed Topics

* **`/terrain_map`** ([grid_map_msgs/GridMap])

	The 2.5D height map with terrain information.

* **`/global_plan`** ([quad_msgs/RobotPlan])

	The global plan consisting of interpolated robot states leading from the start to the goal state.

* **`/state/ground_truth`** ([quad_msgs/RobotState])

	The current state of the robot. 

* **`/cmd_vel`** ([geometry_msgs/Twist])

	Input velocity, given as a twist. 

#### Published Topics

* **`/local_plan`** ([quad_msgs/RobotPlan])

	The local plan, consisting of interpolated robot states. 

* **`/foot_plan_discrete`** ([quad_msgs/MultiFootPlanDiscrete])

	Matrix of foot contact locations (number of contacts x num_legs_). 

* **`/foot_plan_continuous`** ([quad_msgs/MultiFootPlanContinuous])

	Continuous foot trajectories. 

#### Parameters

* **`update_rate`** (int, default: 333)

	The update rate of the controller (in Hz).

* **`timestep`** (double, default: 0.03)

	Timestep duration (in seconds).

* **`horizon_length`** (int, default: 26)

	The length of the planning horizon in number of timesteps. 

* **`desired_height`** (double, default: 0.27)

	Desired z-height (in meters).

* **`toe_radius`** (double, default: 0.02)

	Robot toe radius (in meters).

* **`cmd_vel_filter_const`** (double, default: 0.10)

	Commanded velocity filter constant. 

* **`cmd_vel_scale`** (double, default: 1.0)

	Scale for input twist velocity (cmd_vel).

* **`last_cmd_vel_msg_time_max`** (double, default: 2.0)

	Threshold for maximum amount of time to wait for new twist velocity data from cmd_vel.

* **`stand_vel_threshold`** (double, default: 0.1)

	Velocity threshold to enter stand mode. 

* **`stand_cmd_vel_threshold`** (double, default: 0.1)

	Commanded velocity threshold to enter stand mode. 

* **`stand_pos_error_threshold`** (double, default: 0.05)

	Position error threshold to enter stand mode (measured from foot centroid).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[paper]: https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf
[ros]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[eigen]: http://eigen.tuxfamily.org
[std_srvs/trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
