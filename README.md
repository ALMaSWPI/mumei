There are two ways to run `huron`: using the prebuilt Docker image or building from source.

---

# 1. Running with Docker:

## Prerequisites:
- Docker Engine/Desktop
- Computer with amd64 or arm64 chip

## Steps:
1. Pull the image:
```
sudo docker pull wpihuron/huron:<tag>
```
Currently, `<tag>` can only be a pull request (e.g. `pr-72`).

2. Run the container in interactive mode:
```
sudo docker run -it --network=host wpihuron/huron:<tag>
```
The option `--network=host` is needed to expose the network interfaces (including CAN) to the container.

3. To build and run an example code:

First, `cd` into a sepecific example folder in `examples`, e.g. `examples/test_robot_api`.
Each example code is a normal CMake project. To build the code:
```
mkdir build && cd build
cmake ..
make
```

If everything is correct, the binary will be built in `build` folder, which is ready to be executed.

---

# 2. Building from source:

Clone the main repo and all submodules: 
```
git clone git@github.com:wpi-huron/huron.git --recurse-submodules
```

## Prerequisites:

1. ARM toolchains:
```
sudo apt update
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu
```
2. Build and install third-party CAN library
```
cd third_party/libsockcanpp
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../../tools/<x86_64 or arm64>-toolchain.cmake -DBUILD_SHARED_LIBS=ON
make
sudo make install
```
3. Build and install third-party Serial library
```
cd third_party/serial
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../../../tools/<x86_64 or arm64>-toolchain.cmake -DBUILD_SHARED_LIBS=ON
make
sudo make install
```
4. Build and install Mujoco (if needed)
```
//Install glfw
//Install Mujoco from source. Follow instruction at https://mujoco.readthedocs.io/en/stable/programming/index.html
```
## Build and install:

1. Make sure you are in the root of this repo (`huron/`)
2. Create `build` folder
```
mkdir build
```
3. Build the project
```
cd build
cmake .. [-DBUILD_TYPE=<build-type>] [-DUSE_PINOCCHIO=1]
make
```
4. Install `huron`
```
sudo make install
```

Notes: 

- Currently, the project can be built on Linux only
- By default, the project builds for Raspberry Pi 64-bit (arm64). To change platform,
`BUILD_TYPE` needs to be changed. For example, on Linux x86_64: `-DBUILD_TYPE=x86_64`

---


## Uninstall:

```
cd build
sudo make uninstall
```
