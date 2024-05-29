cmake_minimum_required(VERSION 3.14.0)

project(signaling_server)

# Options and other arguments, required to configure files
set(INSTALL_ON_BUILD ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_INSTALL_ON_BUILD})
set(PIP_DEVEL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_DEVEL_PREFIX})
set(PIP_INSTALL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_PREFIX})
set(USE_EXACT_LIB_SUBPATH ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH})
set(USE_EXACT_LIB_SUBPATH_SOURCE ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH_SOURCE})
set(ENABLE_HTML_DOC ${OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC})
set(PYTHON_HTML_IGNORE_PREFIXES ${PIP_DEVEL_PREFIX} ${PIP_INSTALL_PREFIX})
set(PIP_PYTHON_EXECUTABLE ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PYTHON_EXECUTABLE_SIGNALING_SERVER})

# Needed for pip_utils.cmake
set(PYTHON_SUBPACKAGE_NAME "signaling_server")
set(PYTHON_PACKAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_${PYTHON_SUBPACKAGE_NAME})
set(PYTHON_PACKAGE_CONTENT_DIR ${PYTHON_PACKAGE_DIR}/opentera_webrtc/${PYTHON_SUBPACKAGE_NAME})

include(../cmake/pip_utils.cmake)

set(LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE)
set(BUILD_REQUIREMENTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../requirements.txt)
set(RUN_REQUIREMENTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt)

pip_configure_setup(setup.py.in)

pip_configure_package_file(${LICENSE_FILE} LICENSE)

set(SOURCE
    opentera_webrtc/signaling_server/__init__.py
    opentera_webrtc/signaling_server/room_manager.py
    opentera_webrtc/signaling_server/web_socket_client_manager.py
    opentera_webrtc/signaling_server/signaling_server.py

    opentera-signaling-server
    README.md
    requirements.txt
)
# Same relative tree between CMAKE_CURRENT_SOURCE_DIR and PYTHON_PACKAGE_DIR for files in SOURCE will work as expected with this command
pip_configure_package_file_hierarchy(${SOURCE})

pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/conf.py.in conf.py)
pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/index.rst index.rst)
pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/theme/requirements.txt theme/requirements.txt)

pip_add_requirements_target(NAME ${PYTHON_SUBPACKAGE_NAME}-build-requirements REQUIREMENTS_FILE ${BUILD_REQUIREMENTS_FILE})
pip_add_requirements_target(NAME ${PYTHON_SUBPACKAGE_NAME}-run-requirements REQUIREMENTS_FILE ${RUN_REQUIREMENTS_FILE})
pip_add_html_target(
    NAME ${PYTHON_SUBPACKAGE_NAME}-html
    DEPENDS
        ${PIP_CONFIGURE_OUTPUT_FILES}
        ${${PYTHON_SUBPACKAGE_NAME}-run-requirements}
    DOC_DEPENDS
        ${${PYTHON_SUBPACKAGE_NAME}-build-requirements}
        ${PIP_CONFIGURE_OUTPUT_DOC_FILES}
    ENABLE_HTML_DOC ${ENABLE_HTML_DOC}
)
pip_add_dist_target(
    NAME ${PYTHON_SUBPACKAGE_NAME}-dist
    DEPENDS
        ${${PYTHON_SUBPACKAGE_NAME}-html}
        ${${PYTHON_SUBPACKAGE_NAME}-build-requirements}
)
pip_add_install_target(
    NAME ${PYTHON_SUBPACKAGE_NAME}-install
    DEPENDS
        ${${PYTHON_SUBPACKAGE_NAME}-dist}
    INSTALL_ON_BUILD ${INSTALL_ON_BUILD}
    PIP_DEVEL_PREFIX ${PIP_DEVEL_PREFIX}
    PIP_INSTALL_PREFIX ${PIP_INSTALL_PREFIX}
)
