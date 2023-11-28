# Quad Simulator

## Overview

This package simulates a quadruped robot in Gazebo.  It is heavily based on the [kodlab_gazebo](https://github.com/KodlabPenn/kodlab_gazebo) package.
The included Spirit40 robot model was originally created by Ghost Robotics.  Spirit40 can be simulated either with realistic motor rotor inertia or with rotor inertia neglected.

Important fiction parameters for the foot contact with the ground can be found in the URDF files. xacro file support is a work in progress.

### License

The source code is released under a [MIT License](quad-sdk/LICENSE).

**Original [kodlab_gazebo] Author: Vasileios Vasilopoulos**

**Maintainer: Justin Yim, jkyim@andrew.cmu.edu<br />
Affiliation: [Robomechanics Lab](https://www.cmu.edu/me/robomechanicslab/)**

The Quad Simulator package has been tested under [ROS] Melodic 18.04.
This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Usage

<<<<<<< HEAD
Launch the Gazebo simulation with
=======
8. To launch the simulation, build the package and run:
```
$ roslaunch gazebo_scripts minitaur_gazebo.launch
```
for standalone Minitaur, or
```
$ roslaunch gazebo_scripts minitaur_sensor_gazebo.launch
```
for Minitaur equipped with a sensor head, or 
```
$ roslaunch gazebo_scripts vision60_gazebo.launch
```
for Vision60, or
```
$ roslaunch gazebo_scripts spirit_gazebo.launch
```
for Quad.
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

	roslaunch quad_utils quad_gazebo.launch

## Config files

* **spirit_control.yaml** sets joint controllers and contact publishing rates

## Nodes

### contact_state_publisher

Publishes the Ground Reaction Forces from the feet.

#### Subscribed Topics

<<<<<<< HEAD
* **`/gazebo/toe[0-3]_contact_state`** ([gazebo_msgs/ContactsState])
=======
## Converting Quad's URDF to SDF
We have a URDF xacro file ([spirit_gazebo.urdf.xacro](spirit_description/urdf/spirit_gazebo.urdf.xacro)) in the urdf folder that can be converted to URDF with
```bash
$ python xacro.py spirit_gazebo.urdf.xacro > spirit_gazebo.urdf
```
and subsequently to SDF using
```bash
$ gz sdf -p spirit_gazebo.urdf > ../sdf/spirit.sdf
```
>>>>>>> d5a072b3a89924f1b027bb8b8d27919519fafc18

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