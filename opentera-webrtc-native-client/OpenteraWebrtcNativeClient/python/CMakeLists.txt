cmake_minimum_required(VERSION 3.14.0)

project(opentera_webrtc_native_client)

set(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/bin)

include_directories(../../3rdParty/pybind11/include)
include_directories(../include)
include_directories(include)

file(GLOB_RECURSE
        source_files
        src/*
        include/*
)

if (OPENTERA_WEBRTC_NATIVE_CLIENT_USE_WEBRTC_FOR_JETSON)
    pybind11_add_module(_opentera_webrtc_native_client
            MODULE
            ${source_files}
            NO_EXTRAS
    )
else ()
    pybind11_add_module(_opentera_webrtc_native_client
            MODULE
            ${source_files}
    )
endif ()


target_link_libraries(_opentera_webrtc_native_client PRIVATE
        OpenteraWebrtcNativeClient
        ${webrtc_native_LIBRARY}
        ${PYTHON_LIBRARIES}
)

set_property(TARGET _opentera_webrtc_native_client PROPERTY CXX_STANDARD 14)

assign_source_group(${source_files})

add_subdirectory(package)
