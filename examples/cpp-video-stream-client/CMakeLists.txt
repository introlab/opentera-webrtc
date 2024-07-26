cmake_minimum_required(VERSION 3.14.0)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

project(CppVideoStreamClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/json/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/IXWebSocket)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

add_executable(CppVideoStreamClient main.cpp)

target_link_libraries(CppVideoStreamClient
    OpenteraWebrtcNativeClient
    opencv_videoio
    opencv_highgui
)

if (NOT WIN32)
    target_link_libraries(CppVideoStreamClient
        pthread
    )
endif()

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    add_dependencies(CppVideoStreamClient opencv_highgui opencv_videoio)
endif()

set_property(TARGET CppVideoStreamClient PROPERTY CXX_STANDARD 17)
