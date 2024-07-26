cmake_minimum_required(VERSION 3.14.0)

if (UNIX AND NOT APPLE)
    include_directories(${CMAKE_CURRENT_BINARY_DIR})

    project(CppCameraStreamClient)

    set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(BEFORE SYSTEM ${webrtc_native_INCLUDE})
    include_directories(../../opentera-webrtc-native-client/3rdParty/json/include)
    include_directories(../../opentera-webrtc-native-client/3rdParty/IXWebSocket)
    include_directories(../../opentera-webrtc-native-client/3rdParty/cpp-httplib)
    include_directories(../../opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include)

    add_executable(CppCameraStreamClient main.cpp AlsaPcmDevice.cpp)

    target_link_libraries(CppCameraStreamClient
        OpenteraWebrtcNativeClient
        opencv_videoio
        opencv_highgui
    )


    target_link_libraries(CppCameraStreamClient
        pthread
        asound
    )

    if (NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
        add_dependencies(CppCameraStreamClient opencv_highgui opencv_videoio)
    endif()

    set_property(TARGET CppCameraStreamClient PROPERTY CXX_STANDARD 17)
endif()
