cmake_minimum_required(VERSION 3.14.0)

project(OpenteraWebrtcNativeClientTests)

set(LIBRARY_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})

include_directories(../../3rdParty/googletest/googletest/include)
include_directories(../../3rdParty/googletest/googlemock/include)
include_directories(../../3rdParty/cpp-subprocess)
include_directories(../include)
include_directories(include)


file(GLOB_RECURSE
        TEST_SOURCE_FILES
        "src/*"
        "include/*"
)

add_executable(OpenteraWebrtcNativeClientTests
        ${TEST_SOURCE_FILES}
)

target_link_libraries(OpenteraWebrtcNativeClientTests
        OpenteraWebrtcNativeClient
        Boost::boost
        Boost::filesystem
        gtest
        gmock
)

if (WIN32)
    target_compile_definitions(OpenteraWebrtcNativeClientTests PRIVATE WIN32_LEAN_AND_MEAN)
endif ()

set_property(TARGET OpenteraWebrtcNativeClientTests PROPERTY CXX_STANDARD 14)

assign_source_group(${source_files})

add_custom_command(TARGET OpenteraWebrtcNativeClientTests POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${PROJECT_SOURCE_DIR}/resources $<TARGET_FILE_DIR:OpenteraWebrtcNativeClientTests>/resources
)

install(TARGETS OpenteraWebrtcNativeClientTests DESTINATION bin)
install(DIRECTORY resources DESTINATION bin)
