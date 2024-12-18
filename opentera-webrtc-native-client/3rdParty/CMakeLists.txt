if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

set (BUILD_SHARED_LIBS OFF)

# webrtc_native must be first
add_subdirectory(webrtc_native)

add_subdirectory(pybind11)
add_subdirectory(json)

set(USE_ZLIB OFF)
set(USE_TLS ON)
set(OPENSSL_FOUND TRUE)
set(OPENSSL_DEFINITIONS "")
set(OPENSSL_INCLUDE_DIR ${boringssl_INCLUDE})
set(OPENSSL_LIBRARIES ${boringssl_LIBRARY})
add_subdirectory(IXWebSocket)
set_property(TARGET ixwebsocket PROPERTY CXX_STANDARD 17)
target_compile_options(ixwebsocket PRIVATE -w)

if (OPENTERA_WEBRTC_ENABLE_TESTS)
    add_subdirectory(googletest)
endif()

if(NOT OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    # Set OpenCV flags
    set(OPENCV_FORCE_3RDPARTY_BUILD ON)
    set(BUILD_SHARED_LIBS OFF)

    # Enable/Disable modules :
    set(BUILD_opencv_core ON)
    set(BUILD_opencv_imgproc ON)
    set(BUILD_opencv_calib3d OFF)
    set(BUILD_opencv_dnn OFF)
    set(BUILD_opencv_features2d OFF)
    set(BUILD_opencv_flann OFF)
    set(BUILD_opencv_gapi OFF)
    set(BUILD_opencv_highgui ON)
    set(BUILD_opencv_imgcodecs ON)
    set(BUILD_opencv_ml OFF)
    set(BUILD_opencv_objdetect OFF)
    set(BUILD_opencv_photo OFF)
    set(BUILD_opencv_python2 OFF)
    set(BUILD_opencv_python3 OFF)
    set(BUILD_opencv_stitching OFF)
    set(BUILD_opencv_ts OFF)
    set(BUILD_opencv_video OFF)
    set(BUILD_opencv_videoio ON)

    set(BUILD_JAVA OFF)
    set(BUILD_OBJC OFF)

    set(BUILD_OPENEXR OFF)
    set(WITH_OPENEXR OFF)

    # Configure opencv
    add_subdirectory(opencv)

endif()

install(DIRECTORY json/include DESTINATION include FILES_MATCHING PATTERN "*.hpp")
