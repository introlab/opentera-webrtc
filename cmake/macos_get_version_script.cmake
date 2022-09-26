include(cmake/macos_get_version.cmake)

function(echo)
  execute_process(COMMAND ${CMAKE_COMMAND} -E echo "${ARGN}")
endfunction()

get_macos_sw_vers_product_version()
echo("${SW_VERS_VERSION_SHORT}")
