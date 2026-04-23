# Body Force Estimator

## Overview

This package implements a momentum-based external wrench observer that estimates disturbance torques at the joints and the resulting forces at each toe. It takes the robot state and most recent local plan as input and publishes the estimated external joint torques and toe forces, which can be used by downstream controllers or logged for analysis.

### License

The source code is released under a [MIT License](../LICENSE).

**Affiliation:** [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/), Carnegie Mellon University

Tested under [ROS2] Jazzy on Ubuntu 24.04. This is research code; expect frequent changes and no fitness for any particular purpose.

## Build

```bash
colcon build --packages-select body_force_estimator
```

## Usage

The node is launched as part of the standard robot bringup. To run it standalone against a running robot:

```bash
ros2 run body_force_estimator body_force_estimator_node --ros-args \
    --params-file install/body_force_estimator/share/body_force_estimator/config/body_force_estimator.yaml \
    --params-file install/body_force_estimator/share/body_force_estimator/config/body_force_estimator_topics.yaml
```

## Config

* **`config/body_force_estimator.yaml`** — observer gains and update rate.
* **`config/body_force_estimator_topics.yaml`** — topic remappings.

## Nodes

### body_force_estimator_node

Estimates external joint torques and toe forces from the momentum residual between the predicted dynamics (computed via Pinocchio through `quad_utils::QuadKD2`) and the measured joint state.

#### Subscribed Topics

* **`state/ground_truth`** ([quad_msgs/RobotState])

	Current robot state used to compute the predicted momentum.

* **`local_plan`** ([quad_msgs/LocalPlan])

	Most recent local plan, used to extract the commanded ground reaction forces for the residual computation.

#### Published Topics

* **`body_force/joint_torques`** ([quad_msgs/BodyForceEstimate])

	Estimated external joint torques for all legs.

* **`body_force/toe_forces`** ([quad_msgs/BodyForceEstimate])

	Estimated external forces at each toe, derived from the joint-torque residual via the leg Jacobian.

#### Parameters

* **`body_force_estimator.update_rate`** (double, default: `250.0`) — observer loop rate in Hz.
* **`body_force_estimator.K_O`** (double, default: `25.0`) — observer gain for the momentum residual.
* **`body_force_estimator.cancel_friction`** (int, default: `1`) — subtract the joint friction model from the estimate (`1` = enabled).

## Scripts

* **`scripts/leg_flail.py`** — commands a swinging motion on a single leg for observer testing.
* **`scripts/path_following.py`** — utility script for closed-loop force-disturbance experiments.

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[ROS2]: https://docs.ros.org/en/jazzy/
[quad_msgs/RobotState]: ../quad_msgs/msg/RobotState.msg
[quad_msgs/LocalPlan]: ../quad_msgs/msg/LocalPlan.msg
[quad_msgs/BodyForceEstimate]: ../quad_msgs/msg/BodyForceEstimate.msg
