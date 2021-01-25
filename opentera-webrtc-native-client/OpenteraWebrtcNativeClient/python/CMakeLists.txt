cmake_minimum_required(VERSION 2.6.4)

project(opentera_webrtc_native_client)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(../../3rdParty/pybind11/include)
include_directories(../include)
include_directories(include)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

pybind11_add_module(opentera_webrtc_native_client
        MODULE
        ${source_files})

target_link_libraries(opentera_webrtc_native_client PRIVATE
        OpenteraWebrtcNativeClient
        ${webrtc_native_LIBRARY}
        ${PYTHON_LIBRARIES})

set_property(TARGET opentera_webrtc_native_client PROPERTY CXX_STANDARD 14)

install(TARGETS opentera_webrtc_native_client DESTINATION python/lib)

assign_source_group(${source_files})
