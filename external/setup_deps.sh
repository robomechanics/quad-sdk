quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}
# Setup and Build for IPOPT

#Setup and Build for TeleopTwist
#Setup and Build for RBDL
sudo apt install -y ros-jazzy-urdf
cd rbdl-orb
quiet_mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make -j8
sudo make install
cd ../..