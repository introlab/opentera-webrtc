cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

add_compile_definitions(WEBRTC_POSIX)

find_package(OpenCV REQUIRED core imgproc)
find_package(Boost COMPONENTS system REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIR})
include_directories(../3rdParty/socket.io-client-cpp/src)
include_directories(../3rdParty/socket.io-client-cpp/lib/rapidjson/include)
include_directories(../3rdParty/webrtc_native/webrtc/src)
include_directories(../3rdParty/webrtc_native/webrtc/src/third_party/abseil-cpp)
include_directories(include)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_library(OpenteraWebrtcNativeClient
        STATIC
        ${source_files})

target_link_libraries(OpenteraWebrtcNativeClient
        ssl
        crypto
        sioclient
        sioclient_tls
        ${webrtc_native_LIBRARY}
        ${OpenCV_LIBRARIES}
        ${Boost_LIBRARIES})

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)
add_dependencies(OpenteraWebrtcNativeClient webrtc_native)

assign_source_group(${source_files})

add_subdirectory(python)
add_subdirectory(test)
