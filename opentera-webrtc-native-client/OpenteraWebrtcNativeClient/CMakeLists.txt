cmake_minimum_required(VERSION 2.6.4)

project(OpenteraWebrtcNativeClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_library(OpenteraWebrtcNativeClient
        SHARED
        ${source_files})

target_link_libraries(OpenteraWebrtcNativeClient
        sioclient
        sioclient_tls)

set_property(TARGET OpenteraWebrtcNativeClient PROPERTY CXX_STANDARD 14)

assign_source_group(${source_files})

add_subdirectory(test)
