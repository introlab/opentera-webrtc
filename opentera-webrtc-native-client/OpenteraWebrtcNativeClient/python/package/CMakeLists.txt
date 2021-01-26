# Generate version file

configure_file(setup.py.in setup.py)
configure_file(README.md.in README.md) 

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.py DESTINATION python/package)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/README.md DESTINATION python/package)

