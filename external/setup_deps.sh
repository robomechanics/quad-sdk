quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}

cd mpcplusplus/ext/osqp
quiet_mkdir build
cd build
cmake ..
make -j8
sudo make install
cd ../../../..

cd mpcplusplus/ext/osqp-eigen
quiet_mkdir build
cd build
cmake ..
make -j8
sudo make install
cd ../../../..

# Setup and build for qpOases
cd qpOASES
quiet_mkdir build
cd build
cmake ..
make -j8
sudo make install
cd ../..