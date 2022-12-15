cmake_minimum_required(VERSION 3.14.0)

project(OpenteraWebrtcNativeGStreamer)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_library(${PROJECT_NAME} SHARED ${source_files})
set_property(TARGET OpenteraWebrtcNativeGStreamer PROPERTY CXX_STANDARD 17)

if (OPENTERA_WEBRTC_ENABLE_GSTREAMER)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
    pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
    pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

    # TODO: add as submodule or something
    find_package(fmt REQUIRED)

    target_compile_definitions(OpenteraWebrtcNativeGStreamer PUBLIC USE_GSTREAMER)
    target_include_directories(OpenteraWebrtcNativeGStreamer SYSTEM PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_VIDEO_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})
    target_include_directories(OpenteraWebrtcNativeGStreamer PRIVATE src)
    target_link_libraries(OpenteraWebrtcNativeGStreamer PRIVATE ${webrtc_native_LIBRARY} ${GSTREAMER_LIBRARIES} ${GSTREAMER_VIDEO_LIBRARIES} ${GSTREAMER_APP_LIBRARIES} fmt::fmt)
endif ()
