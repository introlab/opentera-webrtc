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
cd opentera-webrtc
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
    * Do not add all the UNIX tools to path, it will conflict with the `MSYS2` tools. You will be fine if you use the default options.
* [CMake](https://cmake.org/download)
    * Add CMake to the PATH variable for at least the current user.
* [Python 3](https://www.python.org/downloads/windows)
    * Install Python 3.8 or higher
    * Choose customize installation
    * Make sure to include `pip`
    * Make sure to add to `PATH`
    * Make a symlink of the `python.exe` executable as `python3.exe`
        * Find it using `where python` in a CMD prompt
        * Use the command `mklink python3.exe python.exe`
    * If offered to disable PATH length limit, choose to do it
* [MSVC 2019 Build Tools](https://visualstudio.microsoft.com/vs/older-downloads/#visual-studio-2019-and-other-products)
    * Choose the Desktop C++ workload
* `numpy` installed on python 3 with `python3 -m pip install numpy`
* `wheel` installed on python 3 with `python3 -m pip install wheel`
* `pybind11-stubgen` installed on python 3 with `python3 -m pip install pybind11-stubgen`
* `sphinx` installed on python 3 with `python3 -m pip install sphinx`
* [MSYS2](https://www.msys2.org/)
    * Check `Run MSYS2 now` at the end of the installation process
    * Update using `pacman -Syu`
    * Launch `MSYS2 MSYS` from the Start menu
        * Update using `pacman -Su`
    * Add `C:\msys64\usr\bin` and `C:\msys64\usr\local\bin` to `PATH` BEFORE `C:\Windows\System32` (to properly use the `MSYS` commands that have Windows or Git-bash equivalents named the same way)

### Optional: required to run the examples
* [Node.js](https://nodejs.org/)
    * No need to install Python and MSVC Build Tools, as they should be installed from before

### Install MSYS2 packages
These packages are utilities used during the build process
```bash
pacman -S rsync tar perl
```

### Initialize submodules
```bash
git submodule update --init --recursive
```

### Prepare build with cmake
Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)
```powershell
cd opentera-webrtc
mkdir build
cd build
cmake .. -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF
```

### Build
Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)
```powershell
cmake --build . --config Release|Debug
```

### Install
Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)
```powershell
cmake --install .
```
