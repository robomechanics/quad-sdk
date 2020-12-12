quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}

cd osqp
quiet_mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..

cd osqp-eigen
quiet_mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..

cd mpcplusplus
quiet_mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..