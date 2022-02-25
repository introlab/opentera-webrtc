set(PYTHON_LIB_NAME opentera_webrtc_native_client)

set(INSTALL_ON_BUILD ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_INSTALL_ON_BUILD})
set(PIP_DEVEL_OPTIONS ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_DEVEL_OPTIONS})
set(PIP_INSTALL_OPTIONS ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_OPTIONS})

if(INSTALL_ON_BUILD)
    set(ALL_OPTION ALL)
else()
    set(ALL_OPTION)
endif()

set(LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../LICENSE)

# Configuration of cmake template files
configure_file(setup.py.in ${CMAKE_CURRENT_BINARY_DIR}/setup.py.in2)
# configure_file can't use generator expressions
file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/setup.py
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/setup.py.in2
)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md.in ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/README.md)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.in ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/__init__.py)
configure_file(${LICENSE_FILE} ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/LICENSE)

set(GENERATE_CONFIGURE_OUTPUT_FILES
    ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/__init__.py
    ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/README.md
    ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/setup.py
    ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/LICENSE
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/so.md5
    DEPENDS ${PYTHON_LIB_NAME}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PYTHON_LIB_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/$<TARGET_FILE_NAME:${PYTHON_LIB_NAME}>
    COMMAND md5sum ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/$<TARGET_FILE_NAME:${PYTHON_LIB_NAME}> > ${CMAKE_CURRENT_BINARY_DIR}/so.md5
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client
    VERBATIM
)
add_custom_target(
    python-opentera-webrtc-native-client-so
    ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/so.md5
    VERBATIM
)

# Generate html doc
if(OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC)
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/${PYTHON_LIB_NAME}.html
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/so.md5 ${GENERATE_CONFIGURE_OUTPUT_FILES} python-opentera-webrtc-native-client-so
        COMMAND /usr/bin/env python3 -m pydoc -w ${PYTHON_LIB_NAME}
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client
        VERBATIM
    )
    add_custom_target(
        python-opentera-webrtc-native-client-html
        ALL
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/${PYTHON_LIB_NAME}.html
        VERBATIM
    )
    set(HTML_DOC_DEPENDENCY_OPTION python-opentera-webrtc-native-client-html ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/${PYTHON_LIB_NAME}.html)
else()
    set(HTML_DOC_DEPENDENCY_OPTION ${CMAKE_CURRENT_BINARY_DIR}/so.md5 ${GENERATE_CONFIGURE_OUTPUT_FILES} python-opentera-webrtc-native-client-so)
endif()

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/dist.md5
    DEPENDS ${HTML_DOC_DEPENDENCY_OPTION}
    COMMAND /usr/bin/env python3 setup.py bdist bdist_wheel sdist
    COMMAND tar c ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client/dist 2> /dev/null | md5sum > ${CMAKE_CURRENT_BINARY_DIR}/dist.md5
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client
    VERBATIM
)
add_custom_target(
    python-opentera-webrtc-native-client-wheel
    ALL
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/dist.md5
    VERBATIM
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/pip_install.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/pip_install.cmake
    @ONLY
)

add_custom_command(
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/install.md5
    DEPENDS python-opentera-webrtc-native-client-wheel ${CMAKE_CURRENT_BINARY_DIR}/dist.md5 ${CMAKE_CURRENT_BINARY_DIR}/cmake/pip_install.cmake
    COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_OPTIONS=${PIP_DEVEL_OPTIONS} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake/pip_install.cmake
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_native_client
    VERBATIM
)
add_custom_target(
    python-opentera-webrtc-native-client-install
    ${ALL_OPTION}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/install.md5
)

install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_OPTIONS=${PIP_INSTALL_OPTIONS} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake/pip_install.cmake)")
