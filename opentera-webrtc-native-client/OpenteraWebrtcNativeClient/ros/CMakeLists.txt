cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClientRos)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
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

add_executable(OpenteraWebrtcRosTopicStreamer
        src/RosVideoSource.cpp
        src/RosSignalingServerConfiguration.cpp
        src/RosTopicStreamer.cpp)

target_link_libraries(OpenteraWebrtcRosTopicStreamer
        OpenteraWebrtcNativeClient
        ${webrtc_native_LIBRARY}
        ${roscpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_executable(OpenTeraWebrtcRosDataChannelBridge
        src/RosSignalingServerConfiguration.cpp
        src/RosDataChannelBridge.cpp)

target_link_libraries(OpenTeraWebrtcRosDataChannelBridge
        OpenteraWebrtcNativeClient
        ${webrtc_native_LIBRARY}
        ${roscpp_LIBRARIES}
        ${std_msgs_LIBRARIES})