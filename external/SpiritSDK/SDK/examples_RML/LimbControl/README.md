# Limb Control from Xavier or external computer

## What it does
This example shows you how to send limb positions from the Xavier or an external computer (such as your laptop or NUC) to the robot's mainboard, using the Python MAVLink UDP ethernet
interface, and how to receive data back from the mainboard at a high rate.

## Warning
This example makes the robot stand up and down by just setting the target positions of the limbs, on a timed loop, from 0.1m to 0.5m, with gains strong enough to make the robot support its weight. 
If the robot gets a command to extend the legs to 0.5m without getting the previous positions and standing up slowly, it will jump and may fall.
Be careful to only ever start the sending script when the robot is flashed, booted, ready, unkilled and the ethernet connection is plugged in. 
Don't unplug or re-plug the ethernet connection, reflash, or unkill the robot when the script has already started; if you must, wait until it outputs -0.1 toe z position.
If the script is stopped or the ethernet cable is unplugged, your computer freezes, or the robot stops receiving messages for one second for any other reason, the robot will turn off its motors and will fall to the ground. Use a robot harness.

## Install

You'll need CMake 3.12+.

Run ``./installMac`` or ``./installLinux``.

You'll need Python 3.6+

Then, set your ethernet interface to be:

```
address 192.168.168.5
netmask 255.255.255.0
```

(on linux:)
```
sudo vi /etc/network/interfaces
```

## Build and upload
This example is built with CMake, so please create a directory named "build" inside the SDK/examples directory:

```
cd SDK
cd examples
mkdir build
cd build
cmake ..
make
../upload.sh *.bin
```

Where *.bin is the binary file for your robot.

This will build the LimbControl example to binaries in the build directory.

Plug the robot into your computer with the microUSB cable before running ``../upload.sh *.bin``.

The robot is now ready to receive commands via ethernet.

This builds for Vision 60; if you need to build for Spirit, change the ROBOT= line in CMakeLists.txt.

## Run
Plug an ethernet cable from your computer (or from the robot's Xavier) to the mainboard ethernet port, and run:

	cd ..
	cd LimbControl
	./stand

This will start the python main.py file to send MAVLink UDP packets to the robot's mainboard, and the robot will stand up and sit down repeatedly, from commanded toe/limb positions.

This example also shows how to send data back from the mainboard's program to the computer. The received toe z position is printed out to the terminal by the main.py file, along with the mainboard's time since boot. You can add up to 10 floats to send back; see main.cpp for details.
