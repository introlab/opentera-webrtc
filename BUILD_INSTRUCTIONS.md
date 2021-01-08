# Building on Ubuntu 20.04

## Install dependencies

```
sudo apt install ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3
```

## Build

WARNING - This can take a long time and requires about 20Go of free space.

```
mkdir build
cd build
cmake ../
make
```
