Clone the main repo and all submodules: 
```
git clone git@github.com:wpi-huron/huron.git --recurse-submodules
```
---
Prerequisites:
==============

1. ARM toolchains:
```
sudo apt update
sudo apt install gcc-arm-linux-gnueabihf g++-arm-linux-gnueabihf
```
2. Build and install third-party CAN library
```
cd third_party/libsockcanpp
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../../tools/<x86_64 or armhf>-toolchain.cmake -DBUILD_SHARED_LIBS=ON
make
sudo make install
```

Build and install:
==================

1. Make sure you are in the root of this repo (`huron/`)
2. Create `build` folder
```
mkdir build
```
3. Build the project
```
cd build
cmake .. [-DBUILD_TYPE=<build-type>]
make
```
4. Install `huron`
```
sudo make install
```

Notes: 

- Currently, the project can be built on Linux only
- By default, the project builds for Raspberry Pi (armv7l). To change platform,
`BUILD_TYPE` needs to be changed. For example, on Linux x86_64: `-DBUILD_TYPE=x86_64`

---

Uninstall:
==========

```
cd build
sudo make uninstall
```
