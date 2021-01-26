cmake_minimum_required(VERSION 2.6.4)

SET(OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MAJOR "0")
SET(OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MINOR "1")
SET(OPENTERA_WEBRTC_NATIVE_CLIENT_PATCH "0")
SET(OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION ${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MAJOR}.${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION_MINOR}.${OPENTERA_WEBRTC_NATIVE_CLIENT_PATCH})

message("opentera-webrtc-native-client Version : ${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION}")

if (POLICY CMP0014)
    cmake_policy(SET CMP0014 NEW)
endif (POLICY CMP0014)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(opentera-webrtc-native-client)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/dist/${CMAKE_BUILD_TYPE}/opentera-webrtc-native-client-${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION})
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}") 

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if (CMAKE_BUILD_TYPE MATCHES Debug)
    add_definitions(-DDEBUG=1)
endif ()

if (CMAKE_BUILD_TYPE MATCHES Release)
    add_definitions(-DRELEASE=1)
endif ()

if (MSVC)
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")
endif (MSVC)

function(assign_source_group)
    foreach (_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else ()
            set(_source_rel "${_source}")
        endif ()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("${_source_path_msvc}" FILES "${_source}")
    endforeach ()
endfunction(assign_source_group)

if (WIN32)
    #TODO
elseif (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC")
    add_compile_definitions(WEBRTC_POSIX)

    list(APPEND boringssl_INCLUDE "${CMAKE_CURRENT_LIST_DIR}/3rdParty/webrtc_native/webrtc/src/third_party/boringssl/src/include")
    if (CMAKE_BUILD_TYPE MATCHES Debug)
        list(APPEND webrtc_native_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/3rdParty/webrtc_native/webrtc/src/out/Debug/obj/libwebrtc.a" dl X11 expat)
        list(APPEND boringssl_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/3rdParty/webrtc_native/webrtc/src/out/Debug/obj/third_party/boringssl/libboringssl.a")
    endif ()
    if (CMAKE_BUILD_TYPE MATCHES Release)
        list(APPEND webrtc_native_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/3rdParty/webrtc_native/webrtc/src/out/Release/obj/libwebrtc.a" dl X11 expat)
        list(APPEND boringssl_LIBRARY "${CMAKE_CURRENT_LIST_DIR}/3rdParty/webrtc_native/webrtc/src/out/Release/obj/third_party/boringssl/libboringssl.a")
    endif ()
endif (WIN32)

find_package(Python 3 COMPONENTS Interpreter Development REQUIRED)
if (Python_FOUND)
    set(PYBIND11_PYTHON_VERSION ${Python_VERSION})
else()
    message(FATAL_ERROR "Python3 is not found")
endif()

add_subdirectory(3rdParty)
add_subdirectory(OpenteraWebrtcNativeClient)

