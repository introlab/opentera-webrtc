cmake_minimum_required(VERSION 3.14.0)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

project(CppStreamClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/socket.io-client-cpp/src)
include_directories(../../opentera-webrtc-native-client/3rdParty/socket.io-client-cpp/lib/rapidjson/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

add_executable(CppStreamClient main.cpp)

target_link_libraries(CppStreamClient
    OpenteraWebrtcNativeClient
    opencv_highgui
)

if (NOT WIN32)
    target_link_libraries(CppStreamClient
        pthread
    )
endif()

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    add_dependencies(CppStreamClient opencv_highgui)
endif()

set_property(TARGET CppStreamClient PROPERTY CXX_STANDARD 17)
