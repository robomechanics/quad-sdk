# Quad Simulator

## Overview

This package simulates a quadruped robot in Gazebo.  It is heavily based on the [kodlab_gazebo](https://github.com/KodlabPenn/kodlab_gazebo).
This ROS package can be used in conjunction with the Ghost Robotics SDK artifacts, which are independently distributed by Ghost Robotics.

The included Spirit40 robot model was originally created by Ghost Robotics.  Spirit40 can be simulated either with realistic motor rotor inertia or with rotor inertia neglected.

Important fiction parameters for the foot contact with the ground can be found in the URDF and xacro files.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Original [kodlab_gazebo] Author: Vasileios Vasilopoulos**

**Maintainer: Justin Yim, jkyim@andrew.cmu.edu<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)
**

The Quad Simulator package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Usage

Launch the Gazebo simulation with

	roslaunch quad_utils quad_gazebo.launch

## Config files

* **spirit_control.yaml** sets joint controllers and contact publishing rates

## Nodes

### contact_state_publisher

Publishes the Ground Reaction Forces from the feet.

#### Subscribed Topics

* **`/gazebo/toe[0-3]_contact_state`** ([gazebo_msgs/ContactsState])

        These four topics report information about contact at each of the four feet.

#### Published Topics

* **`/state/grfs`** ([quad_msgs/GRFArray])

        Ground reaction forces at all four feet.

#### Parameters

* **`update_rate`** (double, default: 500)

        The update rate of the publisher (in Hz)


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
[std_srvs/Trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/Temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html