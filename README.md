# FakeUSRP

FakeUSRP can using LimeSDR to simulate USRP B210 Device
 
### Installation

You need rebuild UHD Driver

```sh
git clone https://github.com/EttusResearch/uhd.git
cd uhd/host/lib/usrp
git clone https://github.com/jocover/FakeUSRP.git
```
add line to uhd/host/lib/usrp/CMakeLists.txt file
```sh
INCLUDE_SUBDIRECTORY(FakeUSRP)
```

How to rebuild UHD driver: 
[Link](http://files.ettus.com/manual/page_build_guide.html/) 


