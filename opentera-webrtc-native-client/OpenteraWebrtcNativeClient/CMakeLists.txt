cmake_minimum_required(VERSION 3.14.0)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

if (OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    # OpenCV_INCLUDE_DIRS already set in root CMake with find_package(OpenCV)
    set(OpenCV_LIBRARIES ${OpenCV_LIBS} CACHE INTERNAL "")
else ()
    if (OPENTERA_WEBRTC_ENABLE_EXAMPLES)
        set(_OpenCV_include_dirs_examples
                ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/highgui/include
                ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/imgcodecs/include
                )
        set(_OpenCV_libraries_examples opencv_highgui opencv_imgcodecs)
    endif ()

    set(OpenCV_INCLUDE_DIRS
            ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/core/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/imgproc/include
            ${CMAKE_CURRENT_SOURCE_DIR}/../3rdParty/opencv/modules/videoio/include
            ${_OpenCV_include_dirs_examples}
            ${CMAKE_BINARY_DIR} CACHE INTERNAL ""
            )
    set(OpenCV_LIBRARIES opencv_imgproc opencv_core ${_OpenCV_libraries_examples})

    unset(_OpenCV_include_dirs_examples)
    unset(_OpenCV_libraries_examples)
endif ()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
include_directories(BEFORE SYSTEM ${boringssl_INCLUDE})
include_directories(BEFORE SYSTEM ${libyuv_INCLUDE})
include_directories(../3rdParty/json/include)
include_directories(../3rdParty/IXWebSocket)
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
        ixwebsocket
        ${webrtc_native_LIBRARY}
        ${boringssl_LIBRARY}
        ${libyuv_LIBRARY}
        ${OpenCV_LIBRARIES})

if (WIN32)
    target_link_libraries(OpenteraWebrtcNativeClient
            secur32
            winmm
            dmoguids
            wmcodecdspuuid
            msdmo
            strmiids
            iphlpapi
            )
endif ()

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 17)

if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    add_dependencies(OpenteraWebrtcNativeClient ${OpenCV_LIBRARIES})
endif ()


install(TARGETS OpenteraWebrtcNativeClient DESTINATION lib)
install(DIRECTORY include/ DESTINATION include
        FILES_MATCHING PATTERN "*.h")

assign_source_group(${source_files})

add_subdirectory(python)

if (OPENTERA_WEBRTC_ENABLE_TESTS)
    add_subdirectory(test)
endif ()

if (OPENTERA_WEBRTC_ENABLE_GSTREAMER)
    include_directories(../OpenteraWebrtcNativeGStreamer/include)
    target_compile_definitions(OpenteraWebrtcNativeClient PUBLIC USE_GSTREAMER)
    target_link_libraries(OpenteraWebrtcNativeClient OpenteraWebrtcNativeGStreamer)
endif()
