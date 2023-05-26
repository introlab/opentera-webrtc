function(assert_variable_defined NAME)
    if("${${NAME}}" STREQUAL "")
        message(FATAL_ERROR "'${NAME}' not set before including pip_utils.cmake")
    endif()
endfunction()

assert_variable_defined(PYTHON_PACKAGE_DIR)
assert_variable_defined(PYTHON_PACKAGE_CONTENT_DIR)
assert_variable_defined(PIP_PYTHON_EXECUTABLE)

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

function(pip_configure_doc_file in_file out_file)
    configure_file(${in_file} ${PYTHON_PACKAGE_DIR}/_source/${out_file})
    list(APPEND PIP_CONFIGURE_OUTPUT_DOC_FILES ${PYTHON_PACKAGE_DIR}/_source/${out_file})
    set(PIP_CONFIGURE_OUTPUT_DOC_FILES ${PIP_CONFIGURE_OUTPUT_DOC_FILES} PARENT_SCOPE)
endfunction()

function(pip_configure_package_file_hierarchy)
    foreach(file IN ITEMS ${ARGV})
        if(IS_ABSOLUTE ${file})
            message(AUTHOR_WARNING "Files for 'pip_configure_package_file_hierarchy' (here '${file}') should be relative to the package directory ('PYTHON_PACKAGE_DIR') in the same way for input and output")
        endif()
        pip_configure_package_file(${file} ${file})
    endforeach()
endfunction()

function(pip_configure_subpackage_file in_file out_file)
    configure_file(${in_file} ${PYTHON_PACKAGE_CONTENT_DIR}/${out_file})
    list(APPEND PIP_CONFIGURE_OUTPUT_FILES ${PYTHON_PACKAGE_CONTENT_DIR}/${out_file})
    set(PIP_CONFIGURE_OUTPUT_FILES ${PIP_CONFIGURE_OUTPUT_FILES} PARENT_SCOPE)
endfunction()

function(add_dummy_target)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_target(
        ${ARGS_NAME}-target
        ALL
        VERBATIM
    )
    set(${ARGS_NAME} ${ARGS_NAME}-target PARENT_SCOPE)
endfunction()

function(pip_add_requirements_target)
    set(options)
    set(oneValueArgs NAME REQUIREMENTS_FILE)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(
        OUTPUT ${WORKING_DIR}/${ARGS_NAME}-req.stamp
        DEPENDS ${ARGS_REQUIREMENTS_FILE} ${ARGS_DEPENDS}
        COMMAND ${PIP_PYTHON_EXECUTABLE} -m pip install -qq -r ${ARGS_REQUIREMENTS_FILE}
        COMMAND ${CMAKE_COMMAND} -E touch ${WORKING_DIR}/${ARGS_NAME}-req.stamp
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${ARGS_NAME}-target
        ALL
        DEPENDS ${WORKING_DIR}/${ARGS_NAME}-req.stamp
        VERBATIM
    )
    set(${ARGS_NAME} ${ARGS_NAME}-target ${WORKING_DIR}/${ARGS_NAME}-req.stamp PARENT_SCOPE)
endfunction()

function(pip_add_so_target)
    set(options)
    set(oneValueArgs NAME SO_TARGET_NAME)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(
        OUTPUT ${WORKING_DIR}/${ARGS_NAME}.stamp
        DEPENDS ${ARGS_SO_TARGET_NAME} ${ARGS_DEPENDS}
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${ARGS_SO_TARGET_NAME}> ${PYTHON_PACKAGE_CONTENT_DIR}/$<TARGET_FILE_NAME:${ARGS_SO_TARGET_NAME}>
        COMMAND ${CMAKE_COMMAND} -E touch ${WORKING_DIR}/${ARGS_NAME}.stamp
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${ARGS_NAME}-target
        ALL
        DEPENDS ${WORKING_DIR}/${ARGS_NAME}.stamp
        VERBATIM
    )
    set(${ARGS_NAME} ${ARGS_NAME}-target ${WORKING_DIR}/${ARGS_NAME}.stamp PARENT_SCOPE)
endfunction()

function(pip_add_stub_target)
    set(options)
    set(oneValueArgs NAME PACKAGE_NAME ENABLE_SO_STUB)
    set(multiValueArgs DEPENDS STUB_DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(ARGS_ENABLE_SO_STUB)
        # Used via python -m to correctly add current directory to path, but can be checked for installation nonetheless
        assert_program_installed("pybind11-stubgen")
        string(REPLACE "." "/" PACKAGE_PATH ${ARGS_PACKAGE_NAME})

        add_custom_command(
            OUTPUT ${PYTHON_PACKAGE_CONTENT_DIR}/__init__.pyi
            BYPRODUCTS ${PYTHON_PACKAGE_CONTENT_DIR}/py.typed
            DEPENDS ${ARGS_DEPENDS} ${ARGS_STUB_DEPENDS}
            # Required to allow generating a stub from the local version of the package, pybind11_stubgen changes the current directory
            # before importing the module, as seen here: https://github.com/sizmailov/pybind11-stubgen/blob/master/pybind11_stubgen/__init__.py#L944-L945
            COMMAND ${PIP_PYTHON_EXECUTABLE} -c "from pybind11_stubgen import main as gen; import os, sys; sys.path.insert(0, os.path.abspath('.')); gen(['${ARGS_PACKAGE_NAME}'])"
            COMMAND ${CMAKE_COMMAND} -E copy ${PYTHON_PACKAGE_DIR}/stubs/${PACKAGE_PATH}-stubs/__init__.pyi ${PYTHON_PACKAGE_CONTENT_DIR}/__init__.pyi
            COMMAND bash -c "if [ -f post-process-stub.sh ]; then ./post-process-stub.sh ${PYTHON_PACKAGE_CONTENT_DIR}/__init__.pyi; fi"
            COMMAND ${CMAKE_COMMAND} -E touch ${PYTHON_PACKAGE_CONTENT_DIR}/py.typed
            WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
            VERBATIM
        )
        add_custom_target(
            ${ARGS_NAME}-target
            ALL
            DEPENDS ${PYTHON_PACKAGE_CONTENT_DIR}/__init__.pyi
            VERBATIM
        )
        set(${ARGS_NAME} ${ARGS_NAME}-target ${PYTHON_PACKAGE_CONTENT_DIR}/__init__.pyi PARENT_SCOPE)
    else()
        set(${ARGS_NAME} ${ARGS_DEPENDS} PARENT_SCOPE)
    endif()
endfunction()

function(pip_add_html_target)
    set(options)
    set(oneValueArgs NAME ENABLE_HTML_DOC)
    set(multiValueArgs DEPENDS DOC_DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(ARGS_ENABLE_HTML_DOC)
        assert_program_installed("sphinx-build")

        add_custom_command(
            OUTPUT ${WORKING_DIR}/html.stamp
            DEPENDS ${ARGS_DEPENDS} ${ARGS_DOC_DEPENDS}
            COMMAND ${CMAKE_COMMAND} -E make_directory _source/_templates
            COMMAND ${CMAKE_COMMAND} -E make_directory _source/_static
            COMMAND bash -c "if [ -f _source/theme/requirements.txt ]; then ${PIP_PYTHON_EXECUTABLE} -m pip -qq install -t _source/theme -r _source/theme/requirements.txt; fi"
            COMMAND bash -c "${PIP_PYTHON_EXECUTABLE} -m sphinx -v -aE _source _build"
            COMMAND bash -c "if [ -f post-process-doc.sh ]; then ./post-process-doc.sh; fi"
            COMMAND rm -rf ${PYTHON_PACKAGE_DIR}/_doc
            COMMAND bash -c "rsync -a --prune-empty-dirs --include '_*/' --include '_static/**' --include '*.html' --include '*.js' --include '*.css' --exclude '*' _build/ _doc/"
            COMMAND ${CMAKE_COMMAND} -E touch ${WORKING_DIR}/html.stamp
            WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
            VERBATIM
        )
        add_custom_target(
            ${ARGS_NAME}-target
            ALL
            DEPENDS ${WORKING_DIR}/html.stamp
            VERBATIM
        )
        set(${ARGS_NAME} ${ARGS_NAME}-target ${WORKING_DIR}/html.stamp PARENT_SCOPE)
    else()
        set(${ARGS_NAME} ${ARGS_DEPENDS} PARENT_SCOPE)
    endif()
endfunction()

function(pip_add_dist_target)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    add_custom_command(
        OUTPUT ${WORKING_DIR}/dist.stamp
        DEPENDS ${ARGS_DEPENDS}
        COMMAND ${PIP_PYTHON_EXECUTABLE} setup.py -q bdist_wheel sdist
        COMMAND ${CMAKE_COMMAND} -E touch ${WORKING_DIR}/dist.stamp
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${ARGS_NAME}-target
        ALL
        DEPENDS ${WORKING_DIR}/dist.stamp
        VERBATIM
    )
    set(${ARGS_NAME} ${ARGS_NAME}-target ${WORKING_DIR}/dist.stamp PARENT_SCOPE)
endfunction()

function(pip_add_install_target)
    set(options)
    set(oneValueArgs NAME INSTALL_ON_BUILD PIP_DEVEL_PREFIX PIP_INSTALL_PREFIX)
    set(multiValueArgs DEPENDS)
    cmake_parse_arguments(ARGS "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

    if(ARGS_INSTALL_ON_BUILD)
        set(ALL_OPTION ALL)
    else()
        set(ALL_OPTION)
        file(REMOVE ${WORKING_DIR}/install.stamp)
    endif()

    configure_file(
        ${CMAKE_UTILS_DIR}/pip_install.cmake.in
        ${WORKING_DIR}/pip_install.cmake
        @ONLY
    )

    add_custom_command(
        OUTPUT ${WORKING_DIR}/install.stamp
        DEPENDS ${ARGS_DEPENDS} ${WORKING_DIR}/pip_install.cmake
        COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_PREFIX=${ARGS_PIP_DEVEL_PREFIX} -P ${WORKING_DIR}/pip_install.cmake
        WORKING_DIRECTORY ${PYTHON_PACKAGE_DIR}
        VERBATIM
    )
    add_custom_target(
        ${ARGS_NAME}-target
        ${ALL_OPTION}
        DEPENDS ${WORKING_DIR}/install.stamp
        VERBATIM
    )
    set(${ARGS_NAME} ${ARGS_NAME}-target ${WORKING_DIR}/install.stamp PARENT_SCOPE)
    install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -DPIP_INSTALL_PREFIX=${ARGS_PIP_INSTALL_PREFIX} -P ${WORKING_DIR}/pip_install.cmake)")
endfunction()

#######################
## Required programs ##
#######################
function(assert_program_installed PROGRAM)
    find_program(prog-${PROGRAM} ${PROGRAM})
    if(${prog-${PROGRAM}} STREQUAL "prog-${PROGRAM}-NOTFOUND")
        message(FATAL_ERROR "'${PROGRAM}' needs to be installed")
    endif()
endfunction()

assert_program_installed("perl")
assert_program_installed("rsync")
assert_program_installed("tar")
assert_program_installed("bash")
assert_program_installed("find")
