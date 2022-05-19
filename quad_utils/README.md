# Quad Utils

## Overview

This package implements a collection of utility functions to aid quad kinematic calculations, interfacing between RViz and quad-sdk topics, remote heartbeat function, and various launch files.
The detail functionalities of different classes can be found in [Classes and Functions](#classes-and-functions) section. Methods to use various launch files are explained in [Launch Files](#launch-files) section.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Joe Norby<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainers: Alex Stutt (astutt@andrew.cmu.edu) and Qishun Yu (qishuny@andrew.cmu.edu)**

The Quad Utils package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Classes and Functions

- `quad_kd.cpp` - QuadKD is our kinematics and dynamics library to aid in performing those types of calculations. Some methods are written by hand, others implement RBDL as a backend (mostly for Jacobian and inverse dynamics computation).

### QuadKD guide

The general naming convention for coordinate transformations is

```cpp
QuadKD::proximallinkToDistallinkFK/IKCoordFrame();
```

So for example `QuadKD::worldToFootFKWorldFrame();` would give the coordinates of the foot frame origin relative to the world frame origin expressed the world frame for a given set of body and joint positions , while `QuadKD::legbaseToFootIKLegbaseFrame();` would give the joint positions corresponding to a given foot location specified in the legbase frame.

### Alternative way to use a quad_kd function:

Here is another example of computing rotation matrix given roll pitch and yaw using `getRotationMatrix` in `quad_kd.cpp`:

```cpp
// initialize a shared pointer to QuadKD class in header file
std::shared_ptr<quad_utils::QuadKD> quadKD_;

// make a kinematics object in the cpp file
quadKD_ = std::make_shared<quad_utils::QuadKD>();

// call the function
quadKD_->getRotationMatrix(rpy, rot);

```

- `rviz_interface.cpp` - A class for interfacing between RViz and quad-sdk topics.
- `remote_heartbeat.cpp` - A class for implementing a remote heartbeat to publishe stamped messages at a fixed rate.

## Launch Files

Launch files are divided across system functionality:

- `mapping.launch` - launches nodes terrain_map_publisher and grid_map_visualization
- `planning.launch` - launches nodes global_body_planner and local_planner. Takes arg `global_planner` with values of `twist` (default) or `fgmp`, arg `local_planner` with values of `full` (default) and `body`, and arg `mpc_type` of value `nonlinear` (default) or `convex`
- `control.launch` - launches node inverse_dynamics or open_loop_controller. Takes arg `controller` with values of `inverse_dynamics` (default) or `open_loop`
- `estimation.launch` - launches nodes ekf_estimator, contact_detection, and body_force_estimation
- `mocap.launch` - launches node mocap_node with config file and ground_truth_publisher
- `visualization.launch` - launches nodes robot_state_publisher which remaps from imu data, rviz_interface which remaps from /state/ground_truth (can be set to /state/estimate or /state/trajectory), and rviz
- `logging.launch` - launches two record nodes to record two bags - one in `quad_logger/bags/archive/quad_log_<timestamp>.bag`, the other in `quad_logger/bags/quad_log_current.bag`

There are four main high-level launch files, one each for the robot and remote computers, one for gazebo, and one to execute plans:

- `robot_driver.launch` - should be run on the robot and calls `control.launch` and `estimation.launch`. Has args `proxy`, `mocap`, and `logging`, each of which default to false. Setting any of these to true calls the corresponding launch files. It also passes arg `controller` to `control.launch` (default inverse_dynamics)
- `remote_driver.launch` - should be run on the remote computer and calls `mapping.launch` and `visualization.launch`, as well as mblink_converter. Has arg `proxy` (default false) which if true will call `robot_driver.launch` with `proxy:=true` and passing the other args through (`mocap` and `logging`)
- `quad_gazebo.launch` - see the Gazebo Simulator page for details.
- `execute_plan.launch` - Calls `planning.launch` with arg `body_planner` (default global), and `logging.launch` if `logging:=true` (default false)

### Common roslaunch calls:

Launch the simulator with RViz visualization, stand the robot up, then execute a plan while logging:

```
roslaunch quad_utils quad_gazebo.launch
rostopic pub /control/mode std_msgs/UInt8 "data: 1"
roslaunch quad_utils planning.launch global_planner:=twist logging:=true
(for twist control input) rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

If your computer is not that powerful, you can slow down the simulation and relax the MPC solving time constraint by:

1. change real_time_update_rate in the .world file (eg. [flat.world](https://github.com/robomechanics/quad-software/blob/main/quad_simulator/gazebo_scripts/worlds/flat/flat.world))
2. change line 91 and 92 of [nmpc_controller.cpp](https://github.com/robomechanics/quad-software/blob/main/nmpc_controller/src/nmpc_controller.cpp) make it match the actual time step you have

Test the full stack without simulating or logging:

```
roslaunch quad_utils remote_driver.launch proxy:=true
roslaunch quad_utils execute_plan.launch
```

Other calls:

- `roslaunch quad_utils robot_driver.launch mocap:=true` - Start the robot stack with the inverse_dynamics controller and mocap data
- `roslaunch quad_utils remote_driver.launch` Start the remote stack to begin sending commands to the robot and visualizing data
- `roslaunch quad_utils remote_driver.launch proxy:=true` - Test the remote stack with robot absent
- `roslaunch quad_utils remote_driver.launch proxy:=true body_planner:=twist` - Test the full stack with robot absent and with simple twist-based high level body plan
- `roslaunch quad_utils open_loop_robot_driver.launch` - Only launch open_loop_controller and mblink_converter nodes
- `roslaunch quad_utils quad_gazebo.launch gui:=true` - Launch the Gazebo sim and visualize the results (need to call other launch files to get it to run our software).

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[paper]: https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf
[ros]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[eigen]: http://eigen.tuxfamily.org
[std_srvs/trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
