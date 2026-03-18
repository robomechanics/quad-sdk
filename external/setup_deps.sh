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

# Setup and Build for Pinocchio
sudo apt install -y ros-jazzy-pinocchio