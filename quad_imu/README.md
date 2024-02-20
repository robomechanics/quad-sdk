# myahrs_driver

## Overview

This is a driver package for the WITHROBOT's myAHRS+ from http://www.lilliputdirect.com/odroid-myahrs|lilliputdirect and http://www.hardkernel.com/main/products/prdt_info.php?g_code=G141464363369 . The myAHRS+ is a low cost high performance AHRS(Attitude Heading Reference System) with USB/UART/I2C interface. The myAHRS+ board contains a 3-axis 16-bit gyroscope, a 3-axis 16-bit accelerometer and a 3-axis 13-bit magnetometer. The driver should also work with USB port.

### Axes Convention

The myAHRS+ board used NED type. The myahrs_driver contained in this package converts to the frame conventions of ROS (use the east north up (ENU) convention and right hand rule) before publishing the msgs. The driver use the coordinate frame below. Please see http://www.ros.org/reps/rep-0103.html#axis-orientation for more information.

 * x forward
 * y left
 * z up


 * NED type IMU: x-north, y-east, z-down, relative to magnetic north.
 * ENU type IMU: x-east, y-north, z-up, relative to magnetic north.

### Original Source

The original source (not support ROS) is maintained github below and tutorials are on the corresponding wiki page. A 3D visualization test like 3D-box is included in this original source. This package used the myAHRS+ SDK below.

https://github.com/withrobot/myAHRS_plus

## Video

This is a visualization demonstration using RViz.

[![test](http://img.youtube.com/vi/j5v5fKppcQo/0.jpg)](http://www.youtube.com/watch?v=j5v5fKppcQo)

## Installation

Install the package:

```sh
sudo apt-get install ros-indigo-myahrs-driver
```

Install the package from the github:

```sh
cd ~/catkin_ws/src
git clone https://github.com/robotpilot/myahrs_driver.git
cd ~/catkin_ws && catkin_make
```

# Run

Run the driver like so:

```sh
rosrun myahrs_driver myahrs_driver _port:=/dev/ttyACM0
```

or

```sh
roslaunch myahrs_driver myahrs_driver.launch
```

## Nodes

Official ROS documentation can be found on the ROS wiki at:

http://wiki.ros.org/myahrs_driver


## Communication Protocol Manual and Forum

The myAHRS+ protocol can be found here(https://github.com/withrobot/myAHRS_plus/tree/master/tutorial). The Forum for myAHRS+ user can be found here(http://forum.odroid.com/viewforum.php?f=109).


## References

### References for myAHRS+ board

* http://www.withrobot.com/myahrs_plus_en/
* http://www.withrobot.com/?wpdmact=process&did=MTE4LmhvdGxpbms=
* https://github.com/robotpilot/myAHRS_plus

* http://www.hardkernel.com/main/products/prdt_info.php?g_code=G141464363369
* http://www.lilliputdirect.com/odroid-myahrs

### References for convention of axes and unit

* http://www.ros.org/reps/rep-0003.html
* http://www.ros.org/reps/rep-0103.html
* https://github.com/paulbovbel/rep/blob/master/rep-0145.rst

### References for similar IMU packages

* http://wiki.ros.org/um6
* http://wiki.ros.org/razor_imu_9dof
* https://github.com/KristofRobot/razor_imu_9dof

* http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html

