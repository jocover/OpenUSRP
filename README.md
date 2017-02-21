# OpenUSRP

OpenUSRP can using LimeSDR to simulate USRP B210 Device

Build Status
------------

Travis: [![Travis Build Status](https://travis-ci.org/jocover/OpenUSRP.svg?branch=master)](https://travis-ci.org/jocover/OpenUSRP)

AppVeyor: [![Build status](https://ci.appveyor.com/api/projects/status/a0h6dwl3wxngeun0?svg=true)](https://ci.appveyor.com/project/jocover/OpenUSRP)

Dependencies
------------
LimeSuite: [https://github.com/myriadrf/LimeSuite.git][1]

Boost: [http://www.boost.org/users/download/][2]

UHD (optional):[https://github.com/EttusResearch/uhd.git][3]

Installation
----------

LIBUHD mode

```sh
git clone https://github.com/EttusResearch/uhd.git
cd uhd/host/lib/usrp
git clone https://github.com/jocover/OpenUSRP.git
```
add line INCLUDE_SUBDIRECTORY(OpenUSRP) to uhd/host/lib/usrp/CMakeLists.txt file
```sh
echo "INCLUDE_SUBDIRECTORY(OpenUSRP)">>CMakeLists.txt

```
Rebuild UHD driver
[Building and Installing UHD from source][4]

----------
UHD MODULE mode
```sh
git clone https://github.com/jocover/OpenUSRP.git
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

add environment variable to .bashrc
```
echo 'export UHD_MODULE_PATH=/usr/lib/uhd/modules' >> ~/.bashrc 
```
Windows Binarie
---------
x86 version:[OpenUSRP_x86.zip][5]

x64 version:[OpenUSRP_x64.zip][6]


  [1]: https://github.com/myriadrf/LimeSuite.git
  [2]: http://www.boost.org/users/download/
  [3]: https://github.com/EttusResearch/uhd.git
  [4]: http://files.ettus.com/manual/page_build_guide.html
  [5]: https://www.jiangwei.org/download/OpenUSRP_x86.zip
  [6]: https://www.jiangwei.org/download/OpenUSRP_x64.zip