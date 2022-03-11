cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

if (OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    # OpenCV_INCLUDE_DIRS already set in root CMake with find_package(OpenCV)
    set(OpenCV_LIBRARIES ${OpenCV_LIBS} CACHE INTERNAL "")
else()
    set(OpenCV_INCLUDE_DIRS
	    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/core/include
	    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/imgproc/include
	    ${CMAKE_CURRENT_BINARY_DIR}/../ CACHE INTERNAL "")

    set(OpenCV_LIBRARIES opencv_imgproc opencv_core)
endif()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${webrtc_native_INCLUDE})
include_directories(BEFORE SYSTEM ${boringssl_INCLUDE})
include_directories(${libyuv_INCLUDE})
include_directories(../3rdParty/socket.io-client-cpp/src)
include_directories(../3rdParty/socket.io-client-cpp/lib/rapidjson/include)
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
        ${boringssl_LIBRARY}
        ${libyuv_LIBRARY}
        ${OpenCV_LIBRARIES})

if(WIN32)
    target_link_libraries(OpenteraWebrtcNativeClient
        secur32
        winmm
        dmoguids
        wmcodecdspuuid
        msdmo
        strmiids
    )
endif()

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    add_dependencies(OpenteraWebrtcNativeClient ${OpenCV_LIBRARIES})
endif()


install(TARGETS OpenteraWebrtcNativeClient DESTINATION lib)
install(DIRECTORY include/ DESTINATION include
	FILES_MATCHING PATTERN "*.h")

assign_source_group(${source_files})

message(STATUS "PYTHON_EXECUTABLE(OpenteraWebrtcNativeClient): ${PYTHON_EXECUTABLE}")
add_subdirectory(python)

if (OPENTERA_WEBRTC_ENABLE_TESTS)
	add_subdirectory(test)
endif()
