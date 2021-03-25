# Building on Ubuntu 20.04

## Install dependencies

```
sudo apt install ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3 libglib2.0-dev libgtk-3-dev libpulse-dev libasound2-dev
```

## Build

Only Unbuntu 18.04 an 20.04 are supported using pre-built libwebrtc library for x86_64, aarch64 or armhf architecture.

### Initialize submodules
```bash
git submodule update --init --recursive
```

### Prepare build with cmake
```bash
cd opentera-webrtc-native-client
mkdir build
cd build
cmake .. -DOPENTERA_WEBRTC_USE_SYSTEM_OPENCV=ON|OFF -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF
```
### Build
```bash
make
```
