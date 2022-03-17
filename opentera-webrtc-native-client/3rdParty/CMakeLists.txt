if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

set (BUILD_SHARED_LIBS OFF)

# webrtc_native must be first
add_subdirectory(webrtc_native)

if(WIN32)
    set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.zip" CACHE STRING "")
    set(BOOST_URL_SHA256 "85a94ac71c28e59cf97a96714e4c58a18550c227ac8b0388c260d6c717e47a69" CACHE STRING "")
else()
    set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.71.0/source/boost_1_71_0.tar.bz2" CACHE STRING "")
    set(BOOST_URL_SHA256 "d73a8da01e8bf8c7eda40b4c84915071a8c8a0df4a6734537ddde4a8580524ee" CACHE STRING "")
endif()
add_subdirectory(boost-cmake)
add_subdirectory(pybind11)
add_subdirectory(socket.io-client-cpp)


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
    set(BUILD_opencv_imgcodecs OFF)
    set(BUILD_opencv_ml OFF)
    set(BUILD_opencv_objdetect OFF)
    set(BUILD_opencv_photo OFF)
    set(BUILD_opencv_python3 OFF)
    set(BUILD_opencv_stitching OFF)
    set(BUILD_opencv_ts OFF)
    set(BUILD_opencv_video OFF)
    set(BUILD_opencv_videoio OFF)

    set(BUILD_JAVA OFF)
    set(BUILD_OBJC OFF)

    # Configure opencv
    add_subdirectory(opencv)

endif()


install(DIRECTORY socket.io-client-cpp/lib/rapidjson/include/rapidjson DESTINATION include
    	FILES_MATCHING PATTERN "*.h")
