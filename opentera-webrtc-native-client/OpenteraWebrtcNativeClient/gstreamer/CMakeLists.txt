cmake_minimum_required(VERSION 3.14.0)

project(OpenteraWebrtcNativeClientGStreamer)

file(GLOB_RECURSE
        source_files
        src/*
        include/*)

add_library(${PROJECT_NAME} STATIC ${source_files})
set_property(TARGET OpenteraWebrtcNativeClientGStreamer PROPERTY CXX_STANDARD 17)

if (OPENTERA_WEBRTC_ENABLE_GSTREAMER)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
    #    pkg_check_modules(GTK REQUIRED gtk+-3.0)
    pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)
    #    pkg_check_modules(GSTREAMER_AUDIO REQUIRED gstreamer-audio-1.0)
    #    pkg_check_modules(GSTREAMER_PBUTILS REQUIRED gstreamer-pbutils-1.0)
    pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

    target_compile_definitions(OpenteraWebrtcNativeClientGStreamer PUBLIC USE_GSTREAMER)
    target_include_directories(OpenteraWebrtcNativeClientGStreamer SYSTEM PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_VIDEO_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})
    target_include_directories(OpenteraWebrtcNativeClientGStreamer PRIVATE src)
    target_link_libraries(OpenteraWebrtcNativeClientGStreamer PRIVATE ${webrtc_native_LIBRARY} ${GSTREAMER_LIBRARIES} ${GSTREAMER_VIDEO_LIBRARIES} ${GSTREAMER_APP_LIBRARIES})
endif ()