cmake_minimum_required(VERSION 2.6.4)

if (POLICY CMP0048)
    cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

add_subdirectory(googletest)
add_subdirectory(pybind11)
add_subdirectory(socket.io-client-cpp)
add_subdirectory(webrtc_native)
