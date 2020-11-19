cmake_minimum_required(VERSION 3.0.2)
project(opentera_webrtc_ros)

## Set compile options

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-fPIC")
add_compile_definitions(WEBRTC_POSIX)

if(NOT CMAKE_BUILD_TYPE )
  MESSAGE(STATUS "not define build type, set to release" )
  set(CMAKE_BUILD_TYPE Release )
elseif(NOT (${CMAKE_BUILD_TYPE} STREQUAL "Release" OR ${CMAKE_BUILD_TYPE} STREQUAL "Debug" ))
  message(FATAL_ERROR "CMAKE_BUILD_TYPE must be either Release or Debug")
endif()

## Find catkin macros and libraries

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  std_msgs
  sensor_msgs
)

## System dependencies are found with CMake's conventions

find_package(OpenCV REQUIRED core imgproc)

if(NOT DEFINED ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR})
  message(FATAL_ERROR "OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR environnement variable must be configured.")
endif()

if(NOT DEFINED ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_LIB})
  message(FATAL_ERROR "OPEN_TERA_WEBRTC_NATIVE_CLIENT_LIB environnement variable must be configured.")
endif()


###################################
## catkin specific configuration ##
###################################

catkin_package()


###########
## Build ##
###########

## Specify additional locations of header files

include_directories(
  include
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/socket.io-client-cpp/src
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/webrtc_native/webrtc/src
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/webrtc_native/webrtc/src/third_party/abseil-cpp
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Specify additional locations of library files

link_directories(
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/socket.io-client-cpp/build/lib/${CMAKE_BUILD_TYPE}
)

## Declare a C++ executable

add_executable(${PROJECT_NAME}_topic_streamer
  src/RosVideoSource.cpp
  src/RosSignalingServerConfiguration.cpp
  src/RosTopicStreamer.cpp
)

add_executable(${PROJECT_NAME}_data_channel_bridge
  src/RosSignalingServerConfiguration.cpp
  src/RosDataChannelBridge.cpp
)

## Rename C++ executable without prefix

set_target_properties(${PROJECT_NAME}_topic_streamer PROPERTIES OUTPUT_NAME topic_streamer PREFIX "")
set_target_properties(${PROJECT_NAME}_data_channel_bridge PROPERTIES OUTPUT_NAME data_channel_bridge PREFIX "")

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_topic_streamer
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_LIB}
  sioclient
  sioclient_tls
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/webrtc_native/webrtc/src/out/${CMAKE_BUILD_TYPE}/obj/libwebrtc.a dl X11 expat
  ${roscpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${cv_bridge_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_data_channel_bridge
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_LIB}
  sioclient
  sioclient_tls
  $ENV{OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR}/../3rdParty/webrtc_native/webrtc/src/out/${CMAKE_BUILD_TYPE}/obj/libwebrtc.a dl X11 expat
  ${roscpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables

## Mark executables for installation

install(
  TARGETS
    ${PROJECT_NAME}_topic_streamer
    ${PROJECT_NAME}_data_channel_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

