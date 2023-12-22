cmake_minimum_required(VERSION 3.14.0)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

project(CppDataChannelReliabilityTests)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/json/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/IXWebSocket)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

add_executable(CppDataChannelReliabilityTests main.cpp)

target_link_libraries(CppDataChannelReliabilityTests
    OpenteraWebrtcNativeClient
)

if (NOT WIN32)
    target_link_libraries(CppDataChannelReliabilityTests
        pthread
    )
endif()

set_property(TARGET CppDataChannelReliabilityTests PROPERTY CXX_STANDARD 17)
