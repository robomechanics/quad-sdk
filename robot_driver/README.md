# Robot Driver

## Overview
This package handles the hardware interfacing as well as state estimation. The class interfaces with LegController and StateEstimator abstract classes. These classes provide a template for various implementations of controllers and estimators. The current release supports an inverse dynamics leg controller and a complementary filter state estimator that fuses the IMU and Mocap measurements to estimate the body position and velocity. An EKF implementation will be available soon for on-board state estimation. 

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Joe Norby<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainers: Ardalan Tajbakhsh (atajbakh@andrew.cmu.edu) **

The Robot Driver package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## STANDUP TESTS

![Alt text](doc/stand_up_down.gif?raw=true)


### Unit Tests

Run the unit tests with

	catkin run_tests robot_driver

## Usage

Robot driver is called by quad_gazebo.launch and is not meant to be used standalone.  

## Config files

* **robot_driver.yaml** Sets low-level controller and estimator hyperparameters.

## Nodes

### robot_driver

    Robot driver's core functionality is interfacing with the hardware through control and state estimation template classes. In the main loop, the node recieves the most recent local plan, control mode, and computes the updated state using an online state estimator (Complementary filter) as well as the control actions (joint torques) that will be applied to the robot. The control actions are computed using inverse dyanamics from the desired ground reaction forces computed by MPC. Furthermore, robot driver handles the control state machine that decides when to transition between different behaviour modes (For example, safety, sit, stand, etc.).

#### Subscribed Topics

* **`/local_planner`** ([quad_msgs/LocalPlan])

	The current local plan computed by MPC.

* **`/control_mode`** ([])

	The current control mode of the robot.

* **`/single_joint_cmd`** ([])


* **`/remote_heartbeat`** ([])

    Robot heartbeat to check if robot is available. 

* **`/control_restart_flag`** ([grid_map_msgs/GridMap])

    Flag that decides when to restart the controller.


#### Published Topics

* **`/leg_command_array`** ([quad_msgs/LegCommandArray])

	The motor commands for each leg.

* **`/grf_pub`** ([quad_msgs/GRFArray])

    The desired ground reaction forces. 

* **`/robot_heartbeat`** ([std_msgs/Header])


* **`/trajectory_robot_state`** ([quad_msgs/RobotState])
    
    The current state estimate of the robot.

#### Parameters

* **`update_rate`** (int, default: 500)

	The update rate of the low-level control and estimation (in Hz).

* **`publish_rate`** (int, default: 500)

	The number of times to call the robot driver each iteration.

* **`mocap_rate`** (int, default: 360)

    The update rate for mocap measurements (in Hz)

* **`mocap_dropout_threshold`** (double, default: 0.027)

* **`filter_time_constant`** (double, default: 0.01)

* **`filter_time_constant`** (double, default: 0.01)

* **`input_timeout`** (double, default: 0.2)

* **`state_timeout`** (double, default: 0.1)

* **`heartbeat_timeout`** (double, default: 0.2)

* **`sit_kp`** (vector, default: [10 10 10])

    Position gains for sit mode.

* **`sit_kd`** (vector, default: [1 1 1])

    Velocity gains for sit mode.

* **`stand_kp`** (vector, default: [35 35 35])

    Position gains for stand mode.

* **`stand_kd`** (vector, default: [1 1 1])
    
    Velocity gains for stand mode.

* **`stance_kp`** (vector, default: [10 10 10])

    Position gains for stance legs.

* **`stand_kd`** (vector, default: [1 1 1])

    Velocity gains for stance legs.

* **`swing_kp`** (vector, default: [10 10 10])

* **`swing_kd`** (vector, default: [1 1 1])

* **`swing_kp_cart`** (vector, default: [0 0 0]) # N/m

* **`swing_kd_cart`** (vector, default: [0 0 0]) # N/m

* **`safety_kp`** (vector, default: [0 0 0])

* **`safety_kd`** (vector, default: [2 2 2])

* **`stand_joint_angles`** (vector, default: [0.0 0.76 1.52])

* **`sit_joint_angles`** (vector, default: [0.0 0.0 0.0])

# Complementary filter coefficients in state-space form, 
# which can be computed by c2d(1/s*(1-G(s))) and c2d(s*G(s)), 
# where G(s) is a second-order low-pass filter
* **`low_pass_a`** (vector, default: [1.863081589528582 -0.871860350323577 1 0])
* **`low_pass_b`** (vector, default: [2 0])
* **`low_pass_c`** (vector, default: [1.469526956982712, -1.476621472298373])
* **`low_pass_d`** (vector, default: [1.616290836790004])
* **`high_pass_a`** (vector, default: [1.901347508294054, -0.905986628426631, 1, 0])
* **`high_pass_b`** (vector, default: [0.062500000000000, 0])
* **`high_pass_c`** (vector, default: [0.031912805108981 -0.028979795079613])
* **`high_pass_d`** (vector, default: [0.0009996017974875904])

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
