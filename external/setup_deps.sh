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
make -j8
sudo make install
cd ../..

cd osqp-eigen
quiet_mkdir build
cd build
cmake ..
make -j8
sudo make install
cd ../..

# Setup and build for qpOases
cd qpOASES
quiet_mkdir build
cd build
cmake ..
make -j8
sudo make install
cd ../..