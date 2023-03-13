# Building Web Clients

1. Install NPM
2. Run the following commands
```bash
   npm install
   npm run build:umd
```

# Building Native Clients

## Ubuntu
Only Ubuntu 20.04 and 22.04 are supported using pre-built libwebrtc library for x86_64 and aarch64 architectures.

### Install Dependencies

```bash
sudo apt install ninja-build python3-dev cmake build-essential libssl-dev libboost-all-dev libopencv-dev python3-pip python3-venv python3 python-is-python3 python3-sphinx libglib2.0-dev libgtk-3-dev libpulse-dev libasound2-dev tree
```

Add the following dependencies if you want GStreamer support :

```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools
```

Also install Python requirements:

```bash
python -m pip install -r requirements.txt
```

### Initialize Submodules (Ubuntu)

```bash
git submodule update --init --recursive
```

### Prepare Build with CMake (Ubuntu)

```bash
cd opentera-webrtc
mkdir build
cd build
cmake .. -DOPENTERA_WEBRTC_USE_SYSTEM_OPENCV=ON|OFF -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF -DOPENTERA_WEBRTC_ENABLE_GSTREAMER=ON|OFF
```

If find_package(Python) failed, you can specify the python version with the CMake option `OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_VERSION`. 

### Build

```bash
make
```

## MacOS
Only MacOS 11, 12 and 13 are supported using pre-built libwebrtc library for x86_64 architecture.

### Install dependencies

```bash
brew install ninja tree ca-certificates
```

Add the following dependencies if you want GStreamer support :
```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-plugins-ugly gst-libav gst-devtools
```

```bash
python -m pip install -r requirements.txt
```

### Initialize Submodules (MacOS)

```bash
git submodule update --init --recursive
```

### Prepare Build with CMake (MacOS)

```bash
cd opentera-webrtc
mkdir build
cd build
cmake .. -DOPENTERA_WEBRTC_USE_SYSTEM_OPENCV=OFF -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF -DOPENTERA_WEBRTC_ENABLE_GSTREAMER=ON|OFF -DOPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_PREFIX='--user'
```

If find_package(Python) failed, you can specify the python version with the CMake option `OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_VERSION`.

### Build

```bash
cmake --build .
```

## Building on Windows (Experimental)

*WARNING* building on Windows is complicated and may not work.

### Install Windows Build Dependencies

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
* [MSVC 2022 Build Tools](https://visualstudio.microsoft.com/fr/downloads/)
  * Choose the Desktop C++ workload
* `numpy` installed on python 3 with `python3 -m pip install numpy`
* `wheel` installed on python 3 with `python3 -m pip install wheel`
* `pybind11-stubgen` installed on python 3 with `python3 -m pip install pybind11-stubgen`
* `sphinx` installed on python 3 with `python3 -m pip install sphinx`
* Install other Python dependencies with `python -m pip install -r requirements.txt`
* Install signaling server Python dependencies with `python -m pip install -r signaling-server\requirements.txt`
* [MSYS2](https://www.msys2.org/)
  * Check `Run MSYS2 now` at the end of the installation process
  * Update using `pacman -Syu`
  * Launch `MSYS2 MSYS` from the Start menu
    * Update using `pacman -Su`
    * Add `C:\msys64\usr\bin` and `C:\msys64\usr\local\bin` to `PATH` BEFORE `C:\Windows\System32` (to properly use the `MSYS` commands that have Windows or Git-bash equivalents named the same way)


### Optional: Required to Run the Examples

* [Node.js](https://nodejs.org/)
  * No need to install Python and MSVC Build Tools, as they should be installed from before

### Install MSYS2 packages for Windows

These packages are utilities used during the build process

```bash
pacman -S rsync tar perl
```

### Initialize Submodules (Windows)

```bash
git submodule update --init --recursive
```

### Prepare Build with CMake (Windows)

Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)

```powershell
cd opentera-webrtc
mkdir build
cd build
cmake .. -DOPENTERA_WEBRTC_ENABLE_TESTS=ON|OFF
```

### Build (Windows)

Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)

```powershell
cmake --build . --config Release|Debug
```

### Install (Windows)

Using a CMD prompt (not tested with `powershell` or `git-bash` or `msys2` or any other shell)

```powershell
cmake --install .
```
