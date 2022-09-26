if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

set (BUILD_SHARED_LIBS OFF)

# webrtc_native must be first
add_subdirectory(webrtc_native)

if(WIN32)
    set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.zip" CACHE STRING "")
    set(BOOST_URL_SHA256 "e34756f63abe8ac34b35352743f17d061fcc825969a2dd8458264edb38781782" CACHE STRING "")
else()
    set(BOOST_URL "https://boostorg.jfrog.io/artifactory/main/release/1.80.0/source/boost_1_80_0.tar.bz2" CACHE STRING "")
    set(BOOST_URL_SHA256 "1e19565d82e43bc59209a168f5ac899d3ba471d55c7610c677d4ccf2c9c500c0" CACHE STRING "")
endif()
add_subdirectory(boost-cmake)
add_subdirectory(pybind11)
add_subdirectory(socket.io-client-cpp)

if(WIN32)
    target_compile_definitions(sioclient PRIVATE WIN32_LEAN_AND_MEAN)
endif()

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
    set(BUILD_opencv_videoio OFF)

    set(BUILD_JAVA OFF)
    set(BUILD_OBJC OFF)

    # Configure opencv
    add_subdirectory(opencv)

endif()


install(DIRECTORY socket.io-client-cpp/lib/rapidjson/include/rapidjson DESTINATION include
    	FILES_MATCHING PATTERN "*.h")
