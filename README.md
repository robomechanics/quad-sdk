# QUAD SDK ON YLO2 ROBOT

Ylo2 is a (13kg) homemade robot, working on a UP-Xtreme I7 board, a peak FDcan 4 can ports, 

Mjbots power board and BLDC motors/controllers (QDD100),

D435, T265, RPLidar A2, Myahrs+(imu), and a lifepo4 24V-10Ah.

(a mix of black anodized cnc alu, carbon, and 3D printed pcabs parts. 

System                  : ubuntu 20.04 - RT patched

Ros                     : Noetic

adapted from noetic_devel_ekf_clean (quad_sdk_branch)

![Alt text](doc/dock2.jpg?raw=true)

![Alt text](doc/dock1.jpg?raw=true)

![Alt text](doc/dock4.jpg?raw=true)

## Installation :

### Install libs specific to ylo2 robot :

* Mraa lib, to give access to security button via up xtreme i7 gui board :

        sudo add-apt-repository ppa:up-division/mraa
        sudo apt-get update
        sudo apt-get install mraa-tools mraa-examples libmraa2 libmraa-dev libupm-dev libupm2 upm-examples
        sudo apt-get install python3-mraa libmraa-java
 
* PcanBasic lib for Peak can M2 board, to control moteus motors :

        sudo apt-get install gcc
        wget https://www.peak-system.com/quick/BasicLinux
        cd PCAN-Basic_Linux-4.7.0.3/libpcanbasic/pcanbasic      # actual version 4.7.0.3 may change..check !
        make clean & make & sudo make install

## Quad sdk package

Refer to the [Quad-SDK Wiki](https://github.com/robomechanics/quad-sdk/wiki/1.-Getting-Started-with-Quad-SDK) for installation, dependency, and unit testing information. Currently Quad-SDK requires ROS Melodic on Ubuntu 18.04. All other dependencies are installed with the included setup script.

[paper]: https://www.andrew.cmu.edu/user/amj1/papers/Quad_SDK_ICRA_Abstract.pdf

## Run simulator :

https://github.com/elpimous/quad-sdk-ylo2-real-robot/assets/8529940/bd33eb78-51fe-4f2b-ac3f-6e3813accc0a

## Run hardware :

https://github.com/elpimous/quad-sdk-ylo2-real-robot/assets/8529940/74056ca0-ea51-4d98-a201-68586ad16de1

# Steps to run on real : Evaluations to confirm !

roslaunch quad_utils robot_driver.launch

roslaunch quad_utils remote_driver.launch

roslaunch quad_utils planning.launch

rostopic pub -1 /control/mode std_msgs/UInt8 "data: 1" 

No success for now !! 

# in progress ...