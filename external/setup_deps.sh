
# Install latest version of CMake (on top of CMake provided by catkin_make)

# Setup and build for libInterpolate
sudo apt install libeigen3-dev -y
sudo apt install libboost-dev libboost-all-dev -y

cd libInterpolate
mkdir build
cd build
cmake ..
make -j4
sudo make install