
cd osqp
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..

cd osqp-eigen
mkdir build
cd build
cmake ..
make -j4
sudo make install
cd ../..