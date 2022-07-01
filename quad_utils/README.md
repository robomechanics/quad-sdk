# Quad Utils

## Overview

This package implements a collection of utility functions to aid quad kinematic calculations, interfacing between RViz and quad-sdk topics, remote heartbeat function, and various launch files.
The detail functionalities of different classes can be found in [Classes and Functions](#classes-and-functions) section. Methods to use various launch files are explained in the [Wiki].

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

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/robomechanics/quad-sdk/issues).

[paper]: https://www.andrew.cmu.edu/user/amj1/papers/IROS2020_Fast_Global_Motion_Planning.pdf
[ros]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[eigen]: http://eigen.tuxfamily.org
[std_srvs/trigger]: http://docs.ros.org/api/std_srvs/html/srv/Trigger.html
[sensor_msgs/temperature]: http://docs.ros.org/api/sensor_msgs/html/msg/Temperature.html
[Wiki]: https://github.com/robomechanics/quad-sdk/wiki/2.-Using-the-Software
