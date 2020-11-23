# MAVLink examples

## What it does
These examples shows you how to send commands from the Xavier or an external computer (such as your laptop or NUC) to the robot's mainboard, using the MAVLink ethernet/WiFi interface.

# Documentation
See here for a tutorial:
http://ghostrobotics.gitlab.io/docs/MAVLink.html

## Build
These examples are built with CMake, so please create a directory named "build" inside the SDK/examples/MAVLink directory:

```
cd SDK
cd examples
cd MAVLink
mkdir build
cd build
cmake ..
make
./stand
```

This will build the Stand and Walk examples to binaries that run on your computer named `stand` and `walk` in the build directory.

## Run
Plug an ethernet cable from your computer to the robot mainboard's ethernet port, or join the robot's WiFi network, or run this from the robot's Xavier, and run:

	./stand

The robot will stand up, look around, and then after a few seconds, sit down.

Run: 

	./walk

The robot will stand up, walk forward about one robot length, walk backwards, turn left, turn right, then sit down again. If you quit the program at any time, and the robot will stop within two seconds.