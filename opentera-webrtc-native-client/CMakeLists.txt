cmake_minimum_required(VERSION 3.14.0)

file(STRINGS ${CMAKE_CURRENT_SOURCE_DIR}/../VERSION VERSION_FROM_FILE)
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

option(OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC "Generate HTML documentation" ON)


if (OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    find_package(OpenCV REQUIRED core imgproc)
    message(STATUS "OpenCV library status:")
    message(STATUS "    config: ${OpenCV_DIR}")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()

if (POLICY CMP0014)
    cmake_policy(SET CMP0014 NEW)
endif (POLICY CMP0014)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(opentera-webrtc-native-client)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/dist/${CMAKE_BUILD_TYPE}/opentera-webrtc-native-client-${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION})
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

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

function(assign_source_group)
    foreach (_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else ()
            set(_source_rel "${_source}")
        endif ()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach ()
endfunction(assign_source_group)

if (WIN32)
    message(FATAL_ERROR "Windows is not supported yet.")
elseif (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC")
    if (APPLE)
        add_compile_definitions(WEBRTC_MAC)
    else(APPLE)
        add_compile_definitions(WEBRTC_POSIX)
    endif(APPLE)
else()
    message(FATAL_ERROR "Unsupported operating system.")
endif (WIN32)

find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
if (Python_FOUND)
    set(PythonLibsNew_FIND_REQUIRED 0 CACHE INTERNAL "")
    set(PYTHON_EXECUTABLE ${Python_EXECUTABLE} CACHE INTERNAL "")
    set(PYBIND11_PYTHON_VERSION ${Python_VERSION})
else()
    message(FATAL_ERROR "Python3 is not found")
endif()

add_subdirectory(3rdParty)
add_subdirectory(OpenteraWebrtcNativeClient)

