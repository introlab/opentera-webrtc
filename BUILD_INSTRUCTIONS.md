# Building on Ubuntu 20.04

## Install dependencies

```
sudo apt install ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3 libglib2.0-dev libgtk-3-dev libpulse-dev libasound2-dev
```

## Build

WARNING - This can take a long time and requires about 20Go of free space.

### Initialize submodules
```bash
git submodule update --init --recursive
```

### Prepare build with cmake
```bash
cd opentera-webrtc-native-client
mkdir build
cd build
cmake ..
```
### Build
```bash
make
```
