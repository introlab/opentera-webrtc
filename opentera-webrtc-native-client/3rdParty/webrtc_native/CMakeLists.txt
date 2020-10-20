cmake_minimum_required(VERSION 2.6.4)

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
                COMMAND fetch --nohooks webrtc
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
                COMMAND git checkout 1f3ac1f8e0688f8eff467b7dedbdb3f202604aeb
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
        message(FATAL_ERROR "\"generate_ninja_files.bash out/Release\" failed (${gn_webrtc_result})")
    endif()

    add_custom_target(
            webrtc_native ALL
            COMMAND ninja -C ${CMAKE_CURRENT_LIST_DIR}/webrtc/src/out/Debug
            COMMAND ninja -C ${CMAKE_CURRENT_LIST_DIR}/webrtc/src/out/Release
    )

endif (WIN32)