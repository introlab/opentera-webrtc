cmake_minimum_required(VERSION 3.14.0)

file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/VERSION VERSION_FROM_FILE)
string(REPLACE "." ";" VERSION_LIST ${VERSION_FROM_FILE})
list(GET VERSION_LIST 0 OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MAJOR)
list(GET VERSION_LIST 1 OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MINOR)
list(GET VERSION_LIST 2 OPENTERA_WEBRTC_NATIVE_CLIENT_PATCH)

set(OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION ${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MAJOR}.${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MINOR}.${OPENTERA_WEBRTC_NATIVE_CLIENT_PATCH}
CACHE INTERNAL "")

message("opentera-webrtc-native-client Version : ${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION}")

add_definitions(-DOPENTERA_WEBRTC_NATIVE_CLIENT_VERSION="${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION}")

# Default behavior is not to use system OpenCV
option(OPENTERA_WEBRTC_USE_SYSTEM_OPENCV "Use system OpenCV library for linking" OFF)

# If OpenCV is not found, you need to set manually the OpenCV_DIR variable with cmake -DOpenCV_DIR=<path>
# Ex: -DOpenCV_DIR=/lib/x86_64-linux-gnu/cmake/opencv4/

# Default behavior is to enable tests
option(OPENTERA_WEBRTC_ENABLE_TESTS "Build tests" ON)

# Default behavior is to enable examples
option(OPENTERA_WEBRTC_ENABLE_EXAMPLES "Build examples" ON)

# Default behavior is to enable stub file
option(OPENTERA_WEBRTC_ENABLE_PYTHON_SO_STUB "Generate stub file" ON)

# Default behavior is to enable HTML documentation
option(OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC "Generate HTML documentation" ON)

# Default behavior is to install only on "install" target
option(
    OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_INSTALL_ON_BUILD
    "If ON, install the native client python library on build instead of install"
    OFF
    )

# Default behavior is to install to --user in both cases
set(
    OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_DEVEL_PREFIX
    ""
    CACHE STRING
    "Pip install '--prefix' when building. If empty, no prefix is used; if '--user', --user is used"
)
set(OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_PREFIX
    ""
    CACHE STRING
    "Pip install '--prefix' when installing. If empty, no prefix is used; if '--user', --user is used"
)
set(OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH
    ""
    CACHE STRING
    "If specified, allow to use a specific lib subpath folder (ex: 'lib/python3/dist-packages' instead of 'lib/python3.8/site-packages')"
)

if (UNIX AND EXISTS /etc/nv_tegra_release)
    message(STATUS "Use NVIDIA hardware acceleration in WebRTC Native on Jetson platforms")
    set(OPENTERA_WEBRTC_NATIVE_CLIENT_USE_WEBRTC_FOR_JETSON ON)
else()
    set(OPENTERA_WEBRTC_NATIVE_CLIENT_USE_WEBRTC_FOR_JETSON OFF)
endif()

if (POLICY CMP0014)
    cmake_policy(SET CMP0014 NEW)
endif (POLICY CMP0014)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

if (POLICY CMP0094)
    cmake_policy(SET CMP0094 NEW)
endif (POLICY CMP0094)

project(opentera-webrtc)

if (NOT DEFINED CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "")
endif ()

find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
if (Python_FOUND)
    set(PythonLibsNew_FIND_REQUIRED 0 CACHE INTERNAL "")
    set(PYTHON3_EXECUTABLE ${Python_EXECUTABLE} CACHE INTERNAL "")
    set(PYBIND11_PYTHON_VERSION ${Python_VERSION})
else()
    message(FATAL_ERROR "Python3 is not found")
endif()

if (WIN32)
    add_compile_definitions(WEBRTC_WIN NOMINMAX _WIN32_WINNT=0x0A00)
elseif (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC")
    add_compile_definitions(WEBRTC_POSIX)
    if (APPLE)
        add_compile_definitions(WEBRTC_MAC)
    endif(APPLE)
else()
    message(FATAL_ERROR "Unsupported operating system.")
endif (WIN32)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG=1)
endif ()

if (CMAKE_BUILD_TYPE MATCHES Release)
    add_definitions(-DRELEASE=1)
endif ()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif (MSVC)

add_subdirectory(opentera-webrtc-native-client)
add_subdirectory(signaling-server)

if(OPENTERA_WEBRTC_ENABLE_EXAMPLES)
    add_subdirectory(examples/cpp-data-channel-client)
    add_subdirectory(examples/cpp-stream-client)
endif()
