cmake_minimum_required(VERSION 3.14.0)


if (OPENTERA_WEBRTC_USE_SYSTEM_OPENCV)
    if(OPENTERA_WEBRTC_ENABLE_EXAMPLES)
        set(_OpenCV_extra_modules_examples "highgui" "imgcodecs")
    else()
        set(_OpenCV_extra_modules_examples "")
    endif()

    find_package(OpenCV REQUIRED core imgproc ${_OpenCV_extra_modules_examples})
    message(STATUS "OpenCV library status:")
    message(STATUS "    config: ${OpenCV_DIR}")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    libraries: ${OpenCV_LIBS}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

    unset(_OpenCV_extra_modules_examples)
endif()

project(opentera-webrtc-native-client)

set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/dist/${CMAKE_BUILD_TYPE}/opentera-webrtc-native-client-${OPENTERA_WEBRTC_NATIVE_CLIENT_VERSION})
message(STATUS "CMAKE_INSTALL_PREFIX : ${CMAKE_INSTALL_PREFIX}")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

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

add_subdirectory(3rdParty)

# Looked up again in case 3rdParty changed it
find_package(Python ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_VERSION} EXACT COMPONENTS Interpreter Development REQUIRED)
if (Python_FOUND)
    set(PYTHON3_EXECUTABLE ${Python_EXECUTABLE} CACHE INTERNAL "")
else()
    message(FATAL_ERROR "Python3 is not found")
endif()

if (OPENTERA_WEBRTC_ENABLE_GSTREAMER)
    add_subdirectory(OpenteraWebrtcNativeGStreamer)
endif()

add_subdirectory(OpenteraWebrtcNativeClient)
