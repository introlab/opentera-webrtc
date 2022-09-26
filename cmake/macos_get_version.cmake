# Inspired from : https://cmake.org/pipermail/cmake/2007-October/017290.html
function(get_macos_sw_vers_product_version)
    find_program(SW_VERS_EXEC sw_vers)
    if(NOT SW_VERS_EXEC)
        message(FATAL_ERROR "Could not detect sw_vers executable, can not gather required information")
    endif()

    execute_process(COMMAND "${SW_VERS_EXEC}" -productVersion OUTPUT_VARIABLE SW_VERS_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

    string(REGEX REPLACE [[^([0-9]+(\.[0-9]+)?)\.[0-9]+$]] "\\1" SW_VERS_VERSION_SHORT "${SW_VERS_VERSION}")
    set(SW_VERS_VERSION_SHORT "${SW_VERS_VERSION_SHORT}" PARENT_SCOPE)
endfunction()
