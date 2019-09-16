

$ cd ~pi/
$ mkdir work
$ cd work/
$ git clone https://github.com/raspberrypi/userland.git
$ cd user_land
$ mkdir build
$ cd build
$ cmake ../
$ cmake --build .

change USER_LAND environment parameter in CMakeLists.txt
