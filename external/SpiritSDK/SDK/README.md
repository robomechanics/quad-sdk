# Ghost Robotics SDK
This directory includes code to flash firmware onto the mainboard (the "Low-Level SDK"), and code to command the robot over MAVLink/ROS (the "High-Level SDK"), along with associated libraries. Most documentation can be found at the [Ghost Robotics GitLab page](https://ghostrobotics.gitlab.io/docs). Contact jnorby@andrew.cmu.edu with questions.
### Disclaimer
- Never make assumptions about what code is on the mainboard. Before running the High Level examples, make sure the default code is on the mainboard

### Installation and Setup
- Refer to the [GR installation page](https://ghostrobotics.gitlab.io/docs/lowlevel/Installation.html) for instructions to install the SDK, but replace the `artifacts/` directory with this one when following any instructions. Note that it is highly recommended to use Linux for this install - the SDK can be compiled in iOS or Windows, but the process is more fickle, and future RML Spirit code will rely on ROS and only have Linux support. (not sure why we don't just put the instructions here, since it is 2 lines)
```
chmod +x install.sh
./install.sh
```
- Follow the `README.md` instructions in `examples_RML/LimbControl/` to set up the build environment for the Low Level examples. This is part of the workflow to flash custom code to the mainboard.
- Follow the `README.md` instructions in `examples_RML/MAVLink/` to set up the build environment for the High Level examples. This is part of the workflow to communicate with the mainboard.

### Flashing the mainboard
`examples_RML/LimbControl/README.md` contains instructions for how to flash the mainboard with custom code. This requires creating a new folder within the `examples_RML/` directory with your desired source code, and modifying the `examples_RML/CMakeLists.txt` file to compile that source code into a binary file. This binary can be flashed to the robot with the `examples/upload.sh` script - see the aforementioned readme for details on this, and see the other examples (e.g. `FirstStep/` and `FirstHop/`.) for examples of other behaviors. Also please add any custom behaviors to the `examples_RML` directory and not `examples/` to avoid conflicts when GR updates their examples.

What does this mean to have something flashed on the mainboard? Like what is the high level doing? How do you compile this make file? 

Example on compiling and uploading code to the mainboard:
```
cd examples_RML
mkdir build
cd build
cmake ..
make
sudo ../upload.sh *.bin
```

Where *.bin is the binary file for your robot (MAVLinkSpeedTest_mb_SPIRIT_0x12.bin). 

The default mainboard firmware can by flashed by executing `sudo ./flash.sh` within the `firmware/`directory. This script has been modified to automatically select the Spirit firmware rather than Vision.

### Communicating with the mainboard
`examples/MAVLink/speed_test.cpp` provides the clearest example for how to use the mblink protocol to talk to the mainboard. This script should be compiled on the computer you would like to use to communicate with the mainboard. This can be a remote computer as long as it is on the robot's WiFi network and has an IP address 192.168.168.X, where X matches the value in `mblink.setRetry("UPST_ADDRESS", 5);` on line 80 of `examples/MAVLink/speed_test.cpp` (this gives the proper upstream address to the mainboard). If running this on the TX2 as is recommended for high bandwidth applications, make sure X is 105 to match the IP address of the TX2 (192.168.168.105). This data should be sent regardless of mainboard firmware (we should confirm this), altough `examples/MAVLinkSpeedTest` contains mainboard code to send custom data from the mainboard to the TX2.

The `mblink/` directory contains information on the mblink protocol library. Data can be sent to the mainboard with the `sendUser()` function, and retrieved with the `get()` function. See the readme for definitions of the default data payload and how to access individual elements.

### A note on updating CMake
Building the examples requires CMake v3.12, but Ubuntu 18.04 only ships with v3.10 (you can check your version with `cmake -version`). Googling how to upgrade may tell you to do something like`sudo apt remove cmake` - DO NOT DO THIS as this will remove most of any ROS environment set up on your computer.  You can try Kitware's official APT repository but I've generally found this to be unsuccessful. The best solution I've found is to build from source, but instead of simply deleting the old version of cmake (and therefore ROS), using alternatives to point Ubuntu to the new cmake. Command line instructions for this can be found [here](https://www.claudiokuenzler.com/blog/796/install-upgrade-cmake-3.12.1-ubuntu-14.04-trusty-alternatives).

### Common errors
- `arm-none-eabi-gcc is not a full path and was not found in the PATH`: CMake doesn't know where the tool for flashing the mainboard is located. Either you didn't run the `install.sh` script, or weren't connected to the internet when you first did. This script checks if you have a folder named `ghost_robotics` in your home directory, and if not it makes that folder and tries to download the appropriate packages there. If you aren't connected to the internet it'll create that folder but won't populate it even if you rerun the install script after connecting to the internet. If this is the case, just delete the created folder and re-run.

## Overview
Fill out how things interact with eachother. What is needed for a system to work.
### How to SSH into the robot
The NVIDIA TX2 on-board computer IP address will be: 192.168.168.105. To log in, connect to the robot’s WiFi, and run:
```
ssh ghost@192.168.168.105
Password: ghost
```
To run the speed_test example, navigate to ~/SpiritSDK/SDK/examples/MAVLink on the TX2 and run the following commands:
```
mkdir build
cd buil
cmake..
make
./speed_test 
```
When the script runs you should get `MBLink rxstop completed` printed at the end. 

Once logged in, to use ROS, run:

```
source ~/.environment_vars.bash
```
### Example Setup
Show example of flashing something to the mainboard - low level. Then using a high level example to control it.
