# Generate files
configure_file(setup.py.in setup.py)
configure_file(README.md.in README.md) 
configure_file(__init__.py.in __init__.py) 

set (PYTHON_LIB_NAME opentera_webrtc_native_client)

# Generate html doc
add_custom_target(pydoc-html-file ALL
	COMMAND python3 -m pydoc -w opentera_webrtc_native_client
	WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
	DEPENDS opentera_webrtc_native_client)

# Install files
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/setup.py DESTINATION python/package)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/README.md DESTINATION python/package)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py DESTINATION python/package)
install(FILES ../../../../LICENSE DESTINATION python/package)
install(FILES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/opentera_webrtc_native_client.html DESTINATION python/package)

# Reinstall python library (could be better)
install(TARGETS opentera_webrtc_native_client DESTINATION python/package)

# Create python wheel package
add_custom_target(python-wheel-package
	COMMAND python3 setup.py bdist bdist_wheel sdist
	WORKING_DIRECTORY ${CMAKE_INSTALL_PREFIX}/python/package)
	
# Signaling server files
set(signaling_server_source_files
    ../../../../signaling-server/signaling_server.py
    ../../../../signaling-server/room_manager.py
    ../../../../signaling-server/requirements.txt
    ../../../../signaling-server/README.md)
         
install(FILES ${signaling_server_source_files} DESTINATION python/signaling-server)
