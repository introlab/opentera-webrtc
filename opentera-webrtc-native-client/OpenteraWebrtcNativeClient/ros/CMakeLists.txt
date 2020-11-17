cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClientRos)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED core imgproc)

include_directories(../../3rdParty/webrtc_native/webrtc/src)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../include)
include_directories(include)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_executable(OpenteraWebrtcRosTopicStreamer
        ${source_files})

target_link_libraries(OpenteraWebrtcRosTopicStreamer
        OpenteraWebrtcNativeClient
        ${webrtc_native_LIBRARY}
        ${roscpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${OpenCV_LIBRARIES})

set_property(TARGET OpenteraWebrtcNativeClientTests PROPERTY CXX_STANDARD 14)

assign_source_group(${source_files})
