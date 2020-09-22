
# Install latest version of CMake (on top of CMake provided by catkin_make)
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt update
sudo apt install cmake -y

# Setup and build for libInterpolate
sudo apt install libeigen3-dev -y
sudo apt install libboost-dev libboost-all-dev -y

cd libInterpolate
mkdir build
cd build
pwd
cmake ..
make -j4
sudo make install