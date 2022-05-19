# Quad Msgs

## Overview

This package defines messages for describing the robot state, trajectory, and robot plan. The package is dependent on sensor_msgs, std_msgs, nav_msgs, geometry_msgs.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Author: Joe Norby<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)<br />
Maintainers: Qishun Yu (qishuny@andrew.cmu.edu)**

The Quad Msgs package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Messages (.msg)

### State Messages

- `RobotState.msg` : The state is defined as an odometry message made with the following messages:

  - body odometry message (`quad_msgs/BodyState`)
  - JointState message (`sensor_msgs/JointState`) with joint positions and velocities
  - multi-foot state message (`quad_msgs/MultiFootState`)

- `BodyState.msg` : The body state is defined as an pose (`geometry_msgs/Pose`) and twist (`geometry_msgs/Twist`) messages.

- `MultiFootState.msg` : A message to hold the state of all feet of a legged robot. The states of each foot are stored in a vector of `quad_msgs/FootState` messages (0 = front left, 1 = back left, 2 = front right, 3 = back right).

- `FootState.msg` : A message to hold the state of a single foot of a legged robot. The states (position, veclocity, and acceleration) of each foot are stored in a vector of FootState messages.

- `GRFArray.msg` : A message to hold an array of ground reaction forces and their points of application (deprecated).

- `ContactMode.msg` : A message to hold contact states of the robot.

- `LegContactMode.msg` : This is a message to hold contact mode of one leg. A bool is used to determine the contact and the contact forces are stored in (`geometry_msgs/Vector3`)

### Command Messages

- `LegCommandArray.msg` : A message of leg commands (`quad_msgs/LegCommand`) for each leg on quad. The order is in FL, BL, FR, BR.

- `LegCommand.msg` : A message of motor commands (`quad_msgs/MotorCommand`) for each joint on a quad leg(stored as Abd, Hip, Knee).

- `MotorCommand.msg` : A message to hold the desired position, desired velocity, feedforward torques and control gains for a single joint on Quad.

### Robot Trajectory Messages

- `RobotStateTrajectory.msg` : A a message to hold a trajectory of robot states. States in the trajectory are stored in a vector of RobotState messages (`quad_msgs/RobotState`).

### Robot Plan Messages

- `BodyPlan.msg` : A message to hold a quad body plan. The plan is defined as an array of odometry messages (`nav_msgs/Odometry`).

- `LocalPlan.msg` : A message to hold a quad local plan. The plan is defined as an array of odometry messages.

- `MultiFootPlanContinuous.msg` : A message to hold a continuous foot plan for multiple robot feet. The plan is defined as a vector of MultiFootState messages

- `MultiFootPlanDiscrete.msg` : A message to hold a discrete foot plan for multiple robot feet. The plans of each foot are stored in a vector of `FootPlanDiscrete` messages. (0 = front left, 1 = back left, 2 = front right, 3 = back right).

- `FootPlanDiscrete.msg` : A message to hold the discrete foot plan for a single robot foot. The plan is defined as a vector of FootState messages.

- `RobotPlan.msg` : This is a message to hold a robot plan. The plan is defined as an array of odometry messages.

- `RobotPlanDiagnostics.msg` : This is a message to hold local plan diagnostics

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ros]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[eigen]: http://eigen.tuxfamily.org
[std_srvs/trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
