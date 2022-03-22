# OpenteraWebrtcNativeClient

## Initialize submodules

```bash
git submodule update --init --recursive
```

webrtc native will be downloaded by CMake at the next step!

## Prepare build with cmake

```bash
cd opentera-webrtc-native-client
mkdir build && cd build
cmake ..
```

## Install dependencies

### Linux - AMD64

```bash
sudo apt install libboost-all-dev
sudo apt install python-is-python3
./3rdParty/webrtc_native/webrtc/src/build/install-build-deps.sh
```
