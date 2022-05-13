# Global Body Planner

## Overview

This package implements global planning algorithms for agile quadrupedal navigation. The package produces point-to-point plans which guide the robot from its current state to the goal given a map of the terrain. The primary navigation algorithm is an RRT-Connect planner which uses motion primitives to produce long-horizon plans that include flight phases. See the [paper](https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf) for more details on the algorithm.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Joe Norby<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainer: Joe Norby, jnorby@andrew.cmu.edu**

The Global Body Planner package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Unit Tests

Run the unit tests with

	catkin run_tests global_body_planner

## Usage

Run the main node with

	roslaunch quad_utils planning.launch global_planner:=fgmp

## Config files

* **global_body_planner.yaml** Sets planning hyperparameters.

## Nodes

### global_body_planner

Publishes global plans to guide lower layers towards the goal state. The node makes iterative calls to the planner from the current state to the desired goal state, and compares the result to the previous best plan. The new plan is accepted and published if one of the following is true:

1. The current plan reaches the goal and is shorter than the previous plan
2. The current plan gets closer to the goal than the previous plan
3. A new goal state has been set since the previous plan was constructed

Internally the node alternates between two states to promote high-quality and feasible plans. Upon recieving a new goal state the node enters the `RESET` mode, which pauses and replans from the current state for a set amount of time (determined by `startup_delay`), after which it publishes the best plan and enters the `REFINE` mode. While in `REFINE`, it continuously replans from a point on the current plan `max_planning_time` seconds into the future to ensure the new plan will be valid.

#### Subscribed Topics

* **`/start_state`** ([quad_msgs/RobotState])

	The current state of the robot. Typically remapped to `/state/ground_truth`.
  
* **`/goal_state`** ([geometry_msgs/PointStamped])

	The 2.5D height map with terrain information. Typically remapped to `/clicked_point` for RViz interaction.

* **`/terrain_map`** ([grid_map_msgs/GridMap])

	The 2.5D height map with terrain information.

#### Published Topics


* **`/global_plan`** ([quad_msgs/RobotPlan])

	The global plan consisting of interpolated robot states leading from the start to the goal state.
  
* **`/global_plan_discrete`** ([quad_msgs/RobotPlan])

	The discrete global plan consisting of the states within the underlying planning tree.

#### Parameters

* **`update_rate`** (double, default: 20)

	The update rate of the planner (in Hz).

* **`num_calls`** (int, default: 1)

	The number of times to call the planner each iteration.
  
* **`max_planning_time`** (double, default: 0.25)

	The maximum time (in s) the planner is allowed to search in one call, after which the path closest to the goal is returned.
  
* **`startup_delay`** (double, default: 1.5)

	The time (in s) spent replanning while in `RESET` mode before publishing a plan for execution.
  
* **`replanning`** (boolean, default: true)

	Enable or disable replanning.
  
* **`replanning`** (boolean, default: true)

	Enable or disable replanning.
  
* **`dt`** (double, default: 0.03)

	Timestep (in s) used for kinematics checks and interpolation.
  
* **`trapped_buffer_factor`** (int, default: 4)

	Number of timesteps that must be valid for a state-action pair to not be considered trapped.
  
* **`backup_ratio`** (double, default: 0.5, min: 0, max: 1)

	Fraction of state-action pair to back up if an invalid state is detected.
  
* **`num_leap_samples`** (int, default: 10)

	Number of leaps sampled per extend function call.
  
* **`body_traversability_threshold`** (double, default: 0.75)

	Traversability threshold for a feasible body state to avoid regions of poor traversability. Set to zero to disable.
  
* **`contact_traversability_threshold`** (double, default: 0.01)

	Traversability threshold for a feasible estimated contact location to avoid regions of poor traversability. Set to zero to disable.
  
* **`mu`** (double, default: 0.25)

	Friction coefficient.
  
* **`g`** (double, default: 9.81)

	Gravity acceleration.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
