
# Ubuntu 20.04
# https://github.com/introlab/webrtc-native-build/releases/download/4389.742109.0/webrtc-native-build-ubuntu-20.04-amd64-Debug-4389.742109.0.tar.gz
# MD5: 97c217934fb7af3a5eebb7f59f6b59c1
# https://github.com/introlab/webrtc-native-build/releases/download/4389.742109.0/webrtc-native-build-ubuntu-20.04-amd64-Release-4389.742109.0.tar.gz
# MD5:  cd8aa6bb6277b2f93944cc0762afed57
set(archive_name webrtc-native-build-ubuntu-20.04-amd64-Release-4389.742109.0.tar.gz)
set(webrtc_native_build_archive_url "https://github.com/introlab/webrtc-native-build/releases/download/4389.742109.0/${archive_name}")
set(webrtc_native_build_archive_md5 "cd8aa6bb6277b2f93944cc0762afed57")

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name})
    message(STATUS "Downloading : ${webrtc_native_build_archive_url}")
    file(DOWNLOAD 
        ${webrtc_native_build_archive_url}
        ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}
        TIMEOUT 60 #seconds
        EXPECTED_MD5 ${webrtc_native_build_archive_md5}
        TLS_VERIFY ON
        SHOW_PROGRESS)
endif()

# string(REPLACE ".tar.gz" "" extracted_directory_name ${archive_name})
set(extracted_directory_name "webrtc-native-build-4389.742109.0")

if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name})
    message(STATUS "Extracting ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}")
    execute_process(
        COMMAND tar xvzf ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set(webrtc_native_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/include CACHE INTERNAL "doc string")
set(boringssl_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/libboringssl.a CACHE INTERNAL "doc string")
set(webrtc_native_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/libwebrtc.a CACHE INTERNAL "doc string")

