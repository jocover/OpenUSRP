# OpenUSRP

OpenUSRP uses a LimeSDR to simulate a USRP B210 device.

Build Status
------------

Travis: [![Travis Build Status](https://travis-ci.org/jocover/OpenUSRP.svg?branch=master)](https://travis-ci.org/jocover/OpenUSRP)

AppVeyor: [![Build status](https://ci.appveyor.com/api/projects/status/a0h6dwl3wxngeun0?svg=true)](https://ci.appveyor.com/project/jocover/OpenUSRP)

Dependencies
------------

- LimeSuite: [https://github.com/myriadrf/LimeSuite.git][1]
- Boost: [http://www.boost.org/users/download/][2]
- UHD (optional): [https://github.com/EttusResearch/uhd.git][3]

Installation
------------

### LIBUHD mode

```sh
git clone https://github.com/EttusResearch/uhd.git
cd uhd/host/lib/usrp
git clone https://github.com/jocover/OpenUSRP.git
```

Add the line `INCLUDE_SUBDIRECTORY(OpenUSRP)` to the `uhd/host/lib/usrp/CMakeLists.txt` file.

```sh
echo "INCLUDE_SUBDIRECTORY(OpenUSRP)" >> CMakeLists.txt

```

Then, rebuild the UHD driver. See [Building and Installing UHD from source][4].


### UHD MODULE mode
```sh
git clone https://github.com/jocover/OpenUSRP.git
mkdir build && cd build
cmake ..
make -j4
sudo make install
sudo ldconfig
```

After the steps above, add the `UHD_MODULE_PATH` environment variable to your system.

```sh
echo 'export UHD_MODULE_PATH=/usr/lib/uhd/modules' | sudo tee --append /etc/environment
# Alternatively, add it for just a user:
# echo 'export UHD_MODULE_PATH=/usr/lib/uhd/modules' >> ~/.bashrc 
```

Testing
-------

```sh
dave@intel:~$ uhd_find_devices
linux; GNU C++ version 5.4.0 20160609; Boost_105800; UHD_003.010.001.001-release

--------------------------------------------------
-- UHD Device 0
--------------------------------------------------
Device Address:
    type: b200
    product: B210
    module: STREAM
    media: USB 3.0
    name: LimeSDR-USB
    serial: 0009060A02430E1E

```

Windows Binaries
----------------

x86 version:[OpenUSRP_x86.zip][5]

x64 version:[OpenUSRP_x64.zip][6]


  [1]: https://github.com/myriadrf/LimeSuite.git
  [2]: http://www.boost.org/users/download/
  [3]: https://github.com/EttusResearch/uhd.git
  [4]: http://files.ettus.com/manual/page_build_guide.html
  [5]: https://www.jiangwei.org/download/OpenUSRP_x86.zip
  [6]: https://www.jiangwei.org/download/OpenUSRP_x64.zip