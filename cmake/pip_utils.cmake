function(assert_variable_defined NAME)
    if(${${NAME}} STREQUAL "")
        message(FATAL_ERROR "'${NAME}' not set before including pip_utils.cmake")
    endif()
endfunction()

assert_variable_defined(PIP_SUBPACKAGE_NAME)

set(PYTHON_PACKAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_${PYTHON_SUBPACKAGE_NAME})
set(PYTHON_PACKAGE_CONTENT_DIR ${PYTHON_PACKAGE_DIR}/opentera/webrtc/${PYTHON_SUBPACKAGE_NAME})
set(WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR}/cmake/pip_utils)
set(CMAKE_UTILS_DIR ${CMAKE_CURRENT_LIST_DIR})

function(pip_configure_setup in_file)
    # Configuration of cmake template files
    configure_file(${in_file} ${WORKING_DIR}/setup.py.in2)
    # configure_file can't use generator expressions
    file(GENERATE
        OUTPUT ${PYTHON_PACKAGE_DIR}/setup.py
        INPUT ${WORKING_DIR}/setup.py.in2
    )
    list(APPEND PIP_CONFIGURE_OUTPUT_FILES ${PYTHON_PACKAGE_DIR}/setup.py)
    set(PIP_CONFIGURE_OUTPUT_FILES ${PIP_CONFIGURE_OUTPUT_FILES} PARENT_SCOPE)
endfunction()

function(pip_configure_package_file in_file out_file)
    configure_file(${in_file} ${PYTHON_PACKAGE_DIR}/${out_file})
    list(APPEND PIP_CONFIGURE_OUTPUT_FILES ${PYTHON_PACKAGE_DIR}/${out_file})
    set(PIP_CONFIGURE_OUTPUT_FILES ${PIP_CONFIGURE_OUTPUT_FILES} PARENT_SCOPE)
endfunction()

function(pip_configure_subpackage_file in_file out_file)
    configure_file(${in_file} ${PYTHON_PACKAGE_CONTENT_DIR}/${out_file})
    list(APPEND PIP_CONFIGURE_OUTPUT_FILES ${PYTHON_PACKAGE_CONTENT_DIR}/${out_file})
    set(PIP_CONFIGURE_OUTPUT_FILES ${PIP_CONFIGURE_OUTPUT_FILES} PARENT_SCOPE)
endfunction()

function(pip_add_so_target)
    set(options)
    set(oneValueArgs NAME SO_NAME)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(PIP_ADD_SO_TARGET "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(
        OUTPUT ${WORKING_DIR}/so.md5
        DEPENDS ${PIP_ADD_SO_TARGET_SO_NAME} ${PIP_ADD_SO_TARGET_DEPENDS}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PIP_ADD_SO_TARGET_SO_NAME}> ${PYTHON_PACKAGE_CONTENT_DIR}/$<TARGET_FILE_NAME:${PIP_ADD_SO_TARGET_SO_NAME}>
        COMMAND md5sum ${PYTHON_PACKAGE_CONTENT_DIR}/$<TARGET_FILE_NAME:${PIP_ADD_SO_TARGET_SO_NAME}> > ${WORKING_DIR}/so.md5
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${PIP_ADD_SO_TARGET_NAME}-target
        ALL
        DEPENDS ${WORKING_DIR}/so.md5
        VERBATIM
    )
    set(${PIP_ADD_SO_TARGET_NAME} ${PIP_ADD_SO_TARGET_NAME}-target ${WORKING_DIR}/so.md5 PARENT_SCOPE)
endfunction()

function(pip_add_html_target)
    set(options ENABLE_HTML_DOC)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(PIP_ADD_HTML_TARGET "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    # TODO: Add support for namespace packages
    if(PIP_ADD_HTML_TARGET_ENABLE_HTML_DOC)
        # add_custom_command(
        #         OUTPUT ${PYTHON_PACKAGE_DIR}/${PYTHON_LIB_NAME}.html
        #         DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/so.md5 ${GENERATE_CONFIGURE_OUTPUT_FILES} python-opentera-webrtc-native-client-so
        #         COMMAND /usr/bin/env python3 -m pydoc -w opentera.webrtc.native_client
        #         COMMAND /usr/bin/env python3 -m pydoc -w opentera.webrtc.native_client.${PYTHON_LIB_NAME}
        #         WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        #         VERBATIM
        #     )
        #     add_custom_target(
        #     python-opentera-webrtc-native-client-html
        #     ALL
        #     DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/${PYTHON_LIB_NAME}.html
        #     VERBATIM
        # )
        # set(HTML_DOC_DEPENDENCY_OPTION python-opentera-webrtc-native-client-html ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/${PYTHON_LIB_NAME}.html)
    else()
        set(HTML_DOC_DEPENDENCY_OPTION ${PIP_ADD_HTML_TARGET_DEPENDS})
    endif()
    set(${PIP_ADD_HTML_TARGET_NAME} ${HTML_DOC_DEPENDENCY_OPTION} PARENT_SCOPE)
endfunction()

function(pip_add_dist_target)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(PIP_ADD_DIST_TARGET "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(
        OUTPUT ${WORKING_DIR}/dist.md5
        DEPENDS ${PIP_ADD_DIST_TARGET_DEPENDS}
        COMMAND /usr/bin/env python3 setup.py bdist bdist_wheel sdist
        COMMAND tar c ${PYTHON_PACKAGE_DIR}/dist 2> /dev/null | md5sum > ${WORKING_DIR}/dist.md5
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${PIP_ADD_DIST_TARGET_NAME}-target
        ALL
        DEPENDS ${WORKING_DIR}/dist.md5
        VERBATIM
    )
    set(${PIP_ADD_DIST_TARGET_NAME} ${PIP_ADD_DIST_TARGET_NAME}-target ${WORKING_DIR}/dist.md5 PARENT_SCOPE)
endfunction()

function(pip_add_install_target)
    set(options)
    set(oneValueArgs NAME INSTALL_ON_BUILD PIP_DEVEL_PREFIX PIP_INSTALL_PREFIX)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(PIP_ADD_INSTALL_TARGET "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(PIP_ADD_INSTALL_TARGET_INSTALL_ON_BUILD)
        set(ALL_OPTION ALL)
    else()
        set(ALL_OPTION)
        file(REMOVE ${WORKING_DIR}/install.md5)
    endif()

    configure_file(
        ${CMAKE_UTILS_DIR}/pip_install.cmake.in
        ${WORKING_DIR}/pip_install.cmake
        @ONLY
    )

    add_custom_command(
        OUTPUT ${WORKING_DIR}/install.md5
        DEPENDS ${PIP_ADD_INSTALL_TARGET_DEPENDS} ${WORKING_DIR}/pip_install.cmake
        COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_PREFIX=${PIP_ADD_INSTALL_TARGET_PIP_DEVEL_PREFIX} -P ${WORKING_DIR}/pip_install.cmake
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${PIP_ADD_INSTALL_TARGET_NAME}-target
        ${ALL_OPTION}
        DEPENDS ${WORKING_DIR}/install.md5
        VERBATIM
    )
    set(${PIP_ADD_INSTALL_TARGET_NAME} ${PIP_ADD_INSTALL_TARGET_NAME}-target ${WORKING_DIR}/install.md5 PARENT_SCOPE)
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_PREFIX=${PIP_ADD_INSTALL_TARGET_PIP_INSTALL_PREFIX} -P ${WORKING_DIR}/pip_install.cmake)")
endfunction()

#######################
## Required programs ##
#######################
function(assert_program_installed PROGRAM)
    find_program(prog ${PROGRAM})
    if(${prog} STREQUAL "prog-NOTFOUND")
        message(FATAL_ERROR "'${PROGRAM}' needs to be installed")
    endif()
endfunction()

assert_program_installed("perl")
assert_program_installed("python3")
assert_program_installed("rsync")
assert_program_installed("tar")
assert_program_installed("md5sum")
