cmake_minimum_required(VERSION 2.6.4)

if (POLICY CMP0014)
    cmake_policy(SET CMP0014 NEW)
endif (POLICY CMP0014)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

if (WIN32)
    #TODO
elseif (UNIX)
    set(CMAKE_CXX_FLAGS "-fPIC")
    add_compile_definitions(WEBRTC_POSIX)

    if (CMAKE_BUILD_TYPE MATCHES Debug)
        list(APPEND linux_LIBRARY dl X11 expat pthread)
    endif ()
    if (CMAKE_BUILD_TYPE MATCHES Release)
        list(APPEND linux_LIBRARY  dl X11 expat pthread)
    endif ()
endif (WIN32)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_subdirectory(../../opentera-webrtc-native-client ./opentera-webrtc-native-client)

project(CppStreamClient)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(../../opentera-webrtc-native-client/3rdParty/opencv/modules/highgui/include)
include_directories(${webrtc_native_INCLUDE})
include_directories(../../opentera-webrtc-native-client/3rdParty/socket.io-client-cpp/src)
include_directories(../../opentera-webrtc-native-client/3rdParty/socket.io-client-cpp/lib/rapidjson/include)
include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

add_executable(CppStreamClient main.cpp)

target_link_libraries(CppStreamClient
        OpenteraWebrtcNativeClient
        sioclient
        ${webrtc_native_LIBRARY}
        ${boringssl_LIBRARY}
        ${OpenCV_LIBRARIES}
        opencv_highgui)

set_property(TARGET CppStreamClient PROPERTY CXX_STANDARD 14)

