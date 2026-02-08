1. install libosal first 
https://github.com/robert-burger/libosal

mkdir -p build
cd build
cmake -DBUILD_FOR_PLATFORM="POSIX" -DBUILD_SHARED_LIBS=ON ..
make
sudo make install


2. install libethercat
git clone https://github.com/robert-burger/libethercat.git
cd libethercat
./bootstrap.sh
autoreconf -is
./configure
make
sudo make install

mkdir build
cd ./build
cmake -DBUILD_FOR_PLATFORM="POSIX" ..
make