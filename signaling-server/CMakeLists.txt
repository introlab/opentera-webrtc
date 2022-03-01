set(INSTALL_ON_BUILD ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_INSTALL_ON_BUILD})
set(PIP_DEVEL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_DEVEL_PREFIX})
set(PIP_INSTALL_PREFIX ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_INSTALL_PREFIX})
set(USE_EXACT_LIB_SUBPATH ${OPENTERA_WEBRTC_NATIVE_CLIENT_PYTHON_PIP_PREFIX_USE_EXACT_LIB_SUBPATH})
set(ENABLE_HTML_DOC ${OPENTERA_WEBRTC_ENABLE_PYTHON_HTML_DOC})

# TODO: Not working currently for namespace packages
set(ENABLE_HTML_DOC OFF)

set(PYTHON_SUBPACKAGE_NAME "signaling_server")

include(../cmake/pip_utils.cmake)

set(LICENSE_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../LICENSE)

pip_configure_setup(setup.py.in)

pip_configure_package_file(${LICENSE_FILE} LICENSE)
pip_configure_package_file(${CMAKE_CURRENT_SOURCE_DIR}/README.md README.md)
pip_configure_package_file(${CMAKE_CURRENT_SOURCE_DIR}/requirements.txt requirements.txt)

pip_configure_subpackage_file(${CMAKE_CURRENT_SOURCE_DIR}/__init__.py __init__.py)
pip_configure_subpackage_file(${CMAKE_CURRENT_SOURCE_DIR}/room_manager.py room_manager.py)
pip_configure_subpackage_file(${CMAKE_CURRENT_SOURCE_DIR}/signaling_server.py signaling_server.py)

pip_add_html_target(NAME ${PYTHON_SUBPACKAGE_NAME}-html DEPENDS ${PIP_CONFIGURE_OUTPUT_FILES} ENABLE_HTML_DOC ${ENABLE_HTML_DOC})
pip_add_dist_target(NAME ${PYTHON_SUBPACKAGE_NAME}-dist DEPENDS ${${PYTHON_SUBPACKAGE_NAME}-html})
pip_add_install_target(
    NAME ${PYTHON_SUBPACKAGE_NAME}-install
    DEPENDS ${${PYTHON_SUBPACKAGE_NAME}-dist}
    INSTALL_ON_BUILD ${INSTALL_ON_BUILD}
    PIP_DEVEL_PREFIX ${PIP_DEVEL_PREFIX}
    PIP_INSTALL_PREFIX ${PIP_INSTALL_PREFIX}
)
