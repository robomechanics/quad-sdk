quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}

# Setup and build for IPOPT
cd ipopt
sudo apt-get install -y gcc g++ gfortran git patch wget pkg-config liblapack-dev libmetis-dev
if [ -d "./coinbrew" ] 
then
	sudo rm -r ./coinbrew
fi
mkdir coinbrew
cd coinbrew
wget https://raw.githubusercontent.com/coin-or/coinbrew/v1.0/coinbrew
chmod u+x coinbrew
./coinbrew fetch Ipopt --latest-release --no-prompt
cd ..
if [ -d "./coinhsl" ] 
then
    echo "HSL found." 
	cp -r ./coinhsl ./coinbrew/ThirdParty/HSL/coinhsl
else
    echo "Warning: HSL not found."
fi
cd coinbrew
./coinbrew build Ipopt --latest-release --tests none --prefix=/usr/local --no-prompt --parallel-jobs=8
cd ../..

# Setup and build for rbdl
sudo apt install -y ros-noetic-urdf
cd rbdl-orb
quiet_mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make -j8
sudo make install
cd ../..

# Setup for teleop_twist_joy to get dependencies installed
sudo apt install -y ros-noetic-teleop-twist-joy
