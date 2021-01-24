cmake_minimum_required(VERSION 2.6.4)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

add_subdirectory(googletest)
add_subdirectory(pybind11)
add_subdirectory(socket.io-client-cpp)
add_subdirectory(webrtc_native)

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
set(BUILD_opencv_highgui OFF)
set(BUILD_opencv_imgcodecs OFF)
set(BUILD_opencv_ml OFF)
set(BUILD_opencv_objdetect OFF)
set(BUILD_opencv_photo OFF)
set(BUILD_opencv_python3 OFF)
set(BUILD_opencv_stitching OFF)
set(BUILD_opencv_ts OFF)
set(BUILD_opencv_video OFF)
set(BUILD_opencv_videoio OFF)

# Configure opencv 
add_subdirectory(opencv)
