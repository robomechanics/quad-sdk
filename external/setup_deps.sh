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
# package.xml. Both cmake args are required too: DISTRO_ROS gates the branch that
# exports the generated custom-message include dirs, and without it the build
# fails on LIVOX_INTERFACES_INCLUDE_DIRECTORIES-NOTFOUND. Writing them into a
# colcon.pkg applies them to this package only, so a plain `colcon build` from
# the workspace root needs no extra flags and no build is done here.
# Livox-SDK2 itself comes from the devcontainer image, not from here.
sudo apt install -y libpcl-dev
cd livox_ros_driver2
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2 launch
cat > colcon.pkg <<EOF
{
  "cmake-args": ["-DROS_EDITION=ROS2", "-DDISTRO_ROS=${ROS_DISTRO:-jazzy}"]
}
EOF
cd ..

# Setup and Build for Pinocchio
sudo apt install -y ros-jazzy-pinocchio

# Setup dependencies for mocap4ros2_optitrack
# mocap4r2_msgs and mocap4r2 are submodules, built by colcon.
# Install any system rosdeps they need:
cd ..
rosdep install --from-paths external/mocap4ros2_optitrack external/mocap4r2_msgs external/mocap4r2 --ignore-src -r -y
cd external