# eth_data_forwarder
ethernet data forwarder

1. build for j5 platform
mkdir build
cd build
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..
make

2. build for orin platform
mkdir build
cd build
cmake ..
make