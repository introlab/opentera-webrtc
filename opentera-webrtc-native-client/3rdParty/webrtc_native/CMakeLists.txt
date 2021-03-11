cmake_minimum_required(VERSION 3.10.0)

if (WIN32)
    #TODO
elseif (UNIX)
    set(ENV{PATH} $ENV{PATH}:${CMAKE_CURRENT_LIST_DIR}/../depot_tools/)

    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/webrtc")
        execute_process(
                COMMAND mkdir webrtc
                WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
                RESULT_VARIABLE mkdir_webrtc_result
        )
        if(NOT ${mkdir_webrtc_result} EQUAL 0)
            message(FATAL_ERROR "\"mkdir webrtc\" failed (${mkdir_webrtc_result})")
        endif()
    endif()

    if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/webrtc/src")
        execute_process(
                COMMAND fetch webrtc
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/webrtc"
                RESULT_VARIABLE fetch_webrtc_result
        )
        if(NOT ${fetch_webrtc_result} EQUAL 0)
            message(FATAL_ERROR "\"fetch webrtc\" failed (${fetch_webrtc_result})")
        endif()

        execute_process(
                COMMAND gclient sync --with_branch_heads --with_tags
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/webrtc"
                RESULT_VARIABLE gclient_sync_webrtc_result
        )
        if(NOT ${gclient_sync_webrtc_result} EQUAL 0)
            message(FATAL_ERROR "\"gclient sync --with_branch_heads --with_tags\" failed (${gclient_sync_webrtc_result})")
        endif()

        execute_process(
                COMMAND git checkout 742109a89d35d95bba055b036d8dbbe2ebcce6ee
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/webrtc/src"
                RESULT_VARIABLE git_checkout_webrtc_result
        )
        if(NOT ${git_checkout_webrtc_result} EQUAL 0)
            message(FATAL_ERROR "\"git checkout\" failed (${git_checkout_webrtc_result})")
        endif()

        execute_process(
                COMMAND gclient sync --with_branch_heads --with_tags
                WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/webrtc"
                RESULT_VARIABLE gclient_sync_webrtc_result
        )
        if(NOT ${gclient_sync_webrtc_result} EQUAL 0)
            message(FATAL_ERROR "\"gclient sync --with_branch_heads --with_tags\" failed (${gclient_sync_webrtc_result})")
        endif()
    endif()

    execute_process(
            COMMAND "${CMAKE_CURRENT_LIST_DIR}/generate_ninja_files.bash"
            WORKING_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/webrtc/src"
            RESULT_VARIABLE gn_webrtc_result
    )
    if(NOT ${gn_webrtc_result} EQUAL 0)
        message(FATAL_ERROR "\"generate_ninja_files.bash\" failed (${gn_webrtc_result})")
    endif()

    add_custom_target(
            webrtc_native ALL
            COMMAND ninja -C ${CMAKE_CURRENT_LIST_DIR}/webrtc/src/out/${CMAKE_BUILD_TYPE}
    )

    # Manual installation of all dependencies.
    list(GET webrtc_native_LIBRARY 0 webrtc_native_LIBRARY_only)
    install(FILES ${webrtc_native_LIBRARY_only} DESTINATION lib)
    install(FILES ${boringssl_LIBRARY} DESTINATION lib)

    install(DIRECTORY webrtc/src/api DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/rtc_base DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

   install(DIRECTORY webrtc/src/media DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/modules DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/system_wrappers DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/common_video DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/common_audio DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/call DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/p2p DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/logging DESTINATION include
        FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/third_party/abseil-cpp/absl DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

    install(DIRECTORY webrtc/src/third_party/boringssl/src/include/openssl DESTINATION include
    	FILES_MATCHING PATTERN "*.h")

endif (WIN32)
