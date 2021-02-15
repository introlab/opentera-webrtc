cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

add_compile_definitions(WEBRTC_POSIX)

set(OpenCV_INCLUDE_DIRS
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/core/include
	${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/imgproc/include
	${CMAKE_CURRENT_BINARY_DIR}/../)

set(OpenCV_LIBRARIES opencv_imgproc opencv_core)


include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../3rdParty/socket.io-client-cpp/src)
include_directories(../3rdParty/socket.io-client-cpp/lib/rapidjson/include)
include_directories(../3rdParty/webrtc_native/webrtc/src)
include_directories(../3rdParty/webrtc_native/webrtc/src/third_party/abseil-cpp)
include_directories(${boringssl_INCLUDE})
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
        ${webrtc_native_LIBRARY}
        ${OpenCV_LIBRARIES})

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)
add_dependencies(OpenteraWebrtcNativeClient webrtc_native ${OpenCV_LIBRARIES})

install(TARGETS OpenteraWebrtcNativeClient DESTINATION lib)
install(DIRECTORY include/ DESTINATION include
	FILES_MATCHING PATTERN "*.h")

assign_source_group(${source_files})

add_subdirectory(python)
add_subdirectory(test)
