cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

add_compile_definitions(WEBRTC_POSIX)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED core imgproc highgui)


include_directories(../3rdParty/socket.io-client-cpp/src)
include_directories(../3rdParty/webrtc_native/webrtc/src)
include_directories(../3rdParty/webrtc_native/webrtc/src/third_party/abseil-cpp)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_library(OpenteraWebrtcNativeClient
        STATIC
        ${source_files})

target_link_libraries(OpenteraWebrtcNativeClient
        sioclient
        sioclient_tls
        ${webrtc_native_LIBRARY}
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${OpenCV_LIBRARIES})

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)
add_dependencies(OpenteraWebrtcNativeClient webrtc_native)


assign_source_group(${source_files})

add_subdirectory(python)
add_subdirectory(test)
