# Building on Ubuntu 20.04

## Install dependencies

```
sudo apt install ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3 libglib2.0-dev libgtk-3-dev libpulse-dev libasound2-dev
```
```
python -m pip install -r requirements.txt
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


# Building on Windows

## Install dependencies
* [Git](https://git-scm.com/downloads/win)
    * Do not add all the UNIX tools to path, it will conflict with the Windows tools.
* [CMake](https://cmake.org/download)
* [Python 3](https://www.python.org/downloads)
    * Make sure to include `pip`
        * If you installed `python 2` BEFORE, you might need to remove it from `PATH` for the time of the install to install `pip` properly
    * Make sure to include the development files
    * Make a copy of the executable as `python3.exe`
    * Make sure to add both to `PATH`
* [Python 2](https://www.python.org/downloads)
    * Make sure to include `pip`
    * Make a copy of the executable as `python2.exe`
    * Make sure to add both to `PATH`
* [MSVC 2019 Build Tools](https://visualstudio.microsoft.com/vs/older-downloads/#visual-studio-2019-and-other-products)
* `numpy` installed on python 2 with `python2 -m pip install numpy`
* `numpy` installed on python 3 with `python3 -m pip install numpy`
* `wheel` installed on python 3 with `python3 -m pip install wheel`
* `pybind11-stubgen` installed on python 3 with `python3 -m pip install pybind11-stubgen`
* `sphinx` installed on python 3 with `python3 -m pip install sphinx`
* [MSYS2](https://www.msys2.org/)
    * Update using `pacman -Syu`
    * Launch `MSYS2 MSYS` from start menu
        * Update using `pacman -Su`
    * Add `C:\msys64\usr\bin` and `C:\msys64\usr\local\bin` to `PATH` BEFORE `C:\Windows\System32` (to properly use the `MSYS` commands that have windows equivalents named the same way)

### Install MSYS2 packages
These packages are utilities used during the build process
```bash
pacman -S rsync tar rsync perl git
```

### Initialize submodules
```bash
git submodule update --init --recursive
```

### Prepare build with cmake
Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)
```powershell
cd opentera-webrtc-native-client
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release|Debug -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF
```

### Build
Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)
```powershell
cmake --build . --config Release|Debug
```
