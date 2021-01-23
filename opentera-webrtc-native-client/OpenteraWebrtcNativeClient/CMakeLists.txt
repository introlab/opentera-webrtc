cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

add_compile_definitions(WEBRTC_POSIX)

#Now building OpenCV from sources instead
#find_package(OpenCV REQUIRED core imgproc)

set(OpenCV_INCLUDE_DIRS 
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/core/include
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/imgproc/include
	${CMAKE_CURRENT_BINARY_DIR}/../)
		
set(OpenCV_LIBRARIES opencv_imgproc opencv_core)

message(STATUS "CMAKE_CURRENT_BINARY_DIR: ${CMAKE_CURRENT_BINARY_DIR} OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../3rdParty/socket.io-client-cpp/src)
include_directories(../3rdParty/socket.io-client-cpp/lib/rapidjson/include)
include_directories(../3rdParty/webrtc_native/webrtc/src)
include_directories(../3rdParty/webrtc_native/webrtc/src/third_party/abseil-cpp)
include_directories(../3rdParty/webrtc_native/webrtc/src/third_party/boringssl/src/include)
include_directories(../3rdParty/cpp-httplib)
include_directories(include)

add_definitions(-DCPPHTTPLIB_OPENSSL_SUPPORT)

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
        ${OpenCV_LIBRARIES})

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)
add_dependencies(OpenteraWebrtcNativeClient webrtc_native ${OpenCV_LIBRARIES})

assign_source_group(${source_files})

add_subdirectory(python)
add_subdirectory(test)
