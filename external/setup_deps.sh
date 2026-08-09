quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}
# Setup and Build for IPOPT
cd ipopt
sudo apt-get install -y gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
if [ -d "./coinbrew" ] 
then
	sudo rm -r ./coinbrew
fi
mkdir coinbrew
cd coinbrew
wget https://raw.githubusercontent.com/coin-or/coinbrew/v2.0/coinbrew
chmod u+x coinbrew
./coinbrew fetch Ipopt --no-prompt
cd ..
if [ -d "./coinhsl" ] 
then
    echo "HSL found." 
	cp -r ./coinhsl ./coinbrew/ThirdParty/HSL/coinhsl
else
    echo "Warning: HSL not found."
fi
cd coinbrew
./coinbrew build Ipopt --latest-release --tests none --prefix=/usr/local --no-prompt
cd ../..


#Setup and Build for RBDL
sudo apt install -y ros-jazzy-urdf
cd rbdl-orb
quiet_mkdir build
cd build
cmake -D CMAKE_POLICY_VERSION_MINIMUM=3.5 -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make -j8
sudo make install
cd ../..

#Setup and Build for TeleopTwist
sudo apt install -y ros-jazzy-teleop-twist-joy

#Setup and Build for Unitree SDK2
cd unitree_sdk2
quiet_mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
sudo make install
cd ../..

# Setup for Livox Mid-360 ROS 2 driver
# Upstream ships the ROS1 and ROS2 manifests side by side and picks between them
# in build.sh, so colcon cannot see the package until one is in place as
# package.xml. Both cmake args are required: DISTRO_ROS gates the branch that
# exports the generated custom-message include dirs, and without it the build
# fails on LIVOX_INTERFACES_INCLUDE_DIRECTORIES-NOTFOUND. Building it here
# seeds the CMake cache so later plain colcon builds of the workspace work.
# Livox-SDK2 itself comes from the devcontainer image, not from here.
cd livox_ros_driver2
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2 launch
cd ../..
colcon build --packages-select livox_ros_driver2 \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy
cd external

# Setup and Build for Pinocchio
sudo apt install -y ros-jazzy-pinocchio

# Setup dependencies for mocap4ros2_optitrack
# mocap4r2_msgs and mocap4r2 are submodules, built by colcon.
# Install any system rosdeps they need:
cd ..
rosdep install --from-paths external/mocap4ros2_optitrack external/mocap4r2_msgs external/mocap4r2 --ignore-src -r -y
cd external