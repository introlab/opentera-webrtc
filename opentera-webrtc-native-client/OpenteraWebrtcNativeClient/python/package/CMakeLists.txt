cmake_minimum_required(VERSION 3.14.0)

project(opentera_webrtc_native_client_package)

# Options and other arguments, required to configure files
set(PYTHON_LIB_NAME _opentera_webrtc_native_client)
set(INSTALL_ON_BUILD ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_INSTALL_ON_BUILD})
set(PIP_DEVEL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_DEVEL_PREFIX})
set(PIP_INSTALL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_PREFIX})
set(USE_EXACT_LIB_SUBPATH ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH})
set(USE_EXACT_LIB_SUBPATH_SOURCE ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH_SOURCE})
set(ENABLE_HTML_DOC ${OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC})
set(ENABLE_SO_STUB ${OPENTERA_WEBRTC_ENABLE_PYTHON_SO_STUB})
set(ENABLE_GSTREAMER ${OPENTERA_WEBRTC_ENABLE_GSTREAMER})
set(PYTHON_HTML_IGNORE_PREFIXES ${PIP_DEVEL_PREFIX} ${PIP_INSTALL_PREFIX})
set(PIP_PYTHON_EXECUTABLE ${PYTHON3_EXECUTABLE})

if (WIN32)
    set(PYTHON_SO_EXTENSION "pyd")
else ()
    set(PYTHON_SO_EXTENSION "so")
endif ()

# Needed for pip_utils.cmake
set(PYTHON_SUBPACKAGE_NAME "native_client")
set(PYTHON_PACKAGE_DIR ${CMAKE_CURRENT_BINARY_DIR}/opentera_webrtc_${PYTHON_SUBPACKAGE_NAME})
set(PYTHON_PACKAGE_CONTENT_DIR ${PYTHON_PACKAGE_DIR}/opentera_webrtc/${PYTHON_SUBPACKAGE_NAME})

include(../../../../cmake/pip_utils.cmake)

set(LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../LICENSE)
set(REQUIREMENTS_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../requirements.txt)

if (ENABLE_GSTREAMER)
    set(GSTREAMER_TARGET_FILE_NAME "$<TARGET_FILE_NAME:OpenteraWebrtcNativeGStreamer>")
else()
    set(GSTREAMER_TARGET_FILE_NAME "")
endif()

pip_configure_setup(setup.py.in)

pip_configure_package_file(${LICENSE_FILE} LICENSE)
pip_configure_package_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md.in README.md)
pip_configure_package_file(${CMAKE_CURRENT_SOURCE_DIR}/MANIFEST.in.in MANIFEST.in)
pip_configure_package_file(${CMAKE_CURRENT_SOURCE_DIR}/post-process-stub.sh post-process-stub.sh)

pip_configure_subpackage_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py.in __init__.py)

pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/conf.py.in conf.py)
pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/index.rst index.rst)
pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/_source/theme/requirements.txt theme/requirements.txt)
pip_configure_doc_file(${CMAKE_CURRENT_SOURCE_DIR}/post-process-doc.sh ../post-process-doc.sh)

pip_add_so_target(NAME ${PYTHON_LIB_NAME}-so SO_TARGET_NAME ${PYTHON_LIB_NAME})
if (ENABLE_GSTREAMER)
    pip_add_so_target(NAME OpenteraWebrtcNativeGStreamer-so SO_TARGET_NAME OpenteraWebrtcNativeGStreamer)
else()
    add_dummy_target(NAME OpenteraWebrtcNativeGStreamer-so)
endif()

pip_add_requirements_target(NAME ${PYTHON_SUBPACKAGE_NAME}-requirements REQUIREMENTS_FILE ${REQUIREMENTS_FILE})
pip_add_stub_target(
        NAME ${PYTHON_LIB_NAME}-stub
        PACKAGE_NAME "opentera_webrtc.native_client"
        DEPENDS
        ${${PYTHON_LIB_NAME}-so}
        ${${PYTHON_SUBPACKAGE_NAME}-requirements}
        ${PIP_CONFIGURE_OUTPUT_FILES}
        ENABLE_SO_STUB ${ENABLE_SO_STUB}
)
pip_add_html_target(
        NAME ${PYTHON_SUBPACKAGE_NAME}-html
        DEPENDS
        ${${PYTHON_LIB_NAME}-stub}
        ${${PYTHON_SUBPACKAGE_NAME}-requirements}
        DOC_DEPENDS
        ${PIP_CONFIGURE_OUTPUT_DOC_FILES}
        ENABLE_HTML_DOC ${ENABLE_HTML_DOC}
)
pip_add_dist_target(
        NAME ${PYTHON_SUBPACKAGE_NAME}-dist
        DEPENDS
        ${${PYTHON_SUBPACKAGE_NAME}-html}
        ${${PYTHON_SUBPACKAGE_NAME}-requirements}
        ${OpenteraWebrtcNativeGStreamer-so}
)
pip_add_install_target(
        NAME ${PYTHON_SUBPACKAGE_NAME}-install
        DEPENDS
        ${${PYTHON_SUBPACKAGE_NAME}-dist}
        INSTALL_ON_BUILD ${INSTALL_ON_BUILD}
        PIP_DEVEL_PREFIX ${PIP_DEVEL_PREFIX}
        PIP_INSTALL_PREFIX ${PIP_INSTALL_PREFIX}
)
