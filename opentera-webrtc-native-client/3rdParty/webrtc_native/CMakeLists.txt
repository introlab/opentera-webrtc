set(webrtc_native_build_version "6422.8505a9.224" CACHE INTERNAL "")

if(WIN32)
	set(architecture "win64")
	set(archive_stem webrtc-native-build-msvc-2022-${architecture}-${CMAKE_BUILD_TYPE}-${webrtc_native_build_version})
	set(archive_extension "zip")
elseif(UNIX)
	set(archive_extension "tar.gz")
	if(APPLE)
		set(architecture "osx64")

		include(${CMAKE_SOURCE_DIR}/cmake/macos_get_version.cmake)
		get_macos_sw_vers_product_version()

		if (NOT ${SW_VERS_VERSION_SHORT} MATCHES "12" AND NOT ${SW_VERS_VERSION_SHORT} MATCHES "13")
			message(FATAL_ERROR "MacOS version not supported: ${SW_VERS_VERSION_SHORT}, must be 12 or 13.")
		endif()

		set(archive_stem webrtc-native-build-macos-${SW_VERS_VERSION_SHORT}-${architecture}-${CMAKE_BUILD_TYPE}-${webrtc_native_build_version})
	else(APPLE)
		# Taken from : https://stackoverflow.com/questions/26919334/detect-underlying-platform-flavour-in-cmake
		function(get_linux_lsb_release_information)
			find_program(LSB_RELEASE_EXEC lsb_release)
			if(NOT LSB_RELEASE_EXEC)
				message(FATAL_ERROR "Could not detect lsb_release executable, can not gather required information")
			endif()

			execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --id OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
			execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --release OUTPUT_VARIABLE LSB_RELEASE_VERSION_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
			execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --codename OUTPUT_VARIABLE LSB_RELEASE_CODENAME_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)


			string(TOLOWER ${LSB_RELEASE_ID_SHORT} LSB_RELEASE_ID_SHORT)
			set(LSB_RELEASE_ID_SHORT "${LSB_RELEASE_ID_SHORT}" PARENT_SCOPE)
			set(LSB_RELEASE_VERSION_SHORT "${LSB_RELEASE_VERSION_SHORT}" PARENT_SCOPE)
			set(LSB_RELEASE_CODENAME_SHORT "${LSB_RELEASE_CODENAME_SHORT}" PARENT_SCOPE)
		endfunction()

		if (${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "aarch64")
			set(architecture "arm64")
		elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "armv*" OR ${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "armhf")
			set(architecture "arm32")
		elseif(${CMAKE_HOST_SYSTEM_PROCESSOR} MATCHES "x86_64")
			set(architecture "amd64")
		else()
			message(FATAL_ERROR "Architecture not supported: ${CMAKE_HOST_SYSTEM_PROCESSOR}")
		endif()

		get_linux_lsb_release_information()

		if (NOT ${LSB_RELEASE_ID_SHORT} MATCHES "ubuntu")
			message(FATAL_ERROR "System not supported: ${LSB_RELEASE_ID_SHORT}, must be ubuntu")
		endif()

		if (NOT ${LSB_RELEASE_VERSION_SHORT} MATCHES "20.04" AND NOT ${LSB_RELEASE_VERSION_SHORT} MATCHES "22.04" AND NOT ${LSB_RELEASE_VERSION_SHORT} MATCHES "24.04")
			message(FATAL_ERROR "Ubuntu version not supported: ${LSB_RELEASE_VERSION_SHORT}, must be 20.04 or 22.04 or 24.04.")
		endif()

		set(archive_stem webrtc-native-build-${LSB_RELEASE_ID_SHORT}-${LSB_RELEASE_VERSION_SHORT}-${architecture}-${CMAKE_BUILD_TYPE}-${webrtc_native_build_version})
	endif(APPLE)
else()
    message(FATAL_ERROR "Unsupported operating system.")
endif()

set(archive_name ${archive_stem}.${archive_extension})

set(webrtc_native_build_archive_url "https://github.com/introlab/webrtc-native-build/releases/download/${webrtc_native_build_version}/${archive_name}")

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${archive_name})
	message(STATUS "Downloading : ${webrtc_native_build_archive_url}")
	file(DOWNLOAD
		${webrtc_native_build_archive_url}
		${CMAKE_CURRENT_BINARY_DIR}/${archive_name}
		TIMEOUT 300 #seconds
		TLS_VERIFY ON
		SHOW_PROGRESS)
endif()

set(extracted_directory_name "webrtc-native-build-${webrtc_native_build_version}-${CMAKE_BUILD_TYPE}")

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name})
	message(STATUS "Extracting ${CMAKE_BINARY_SOURCE_DIR}/${archive_name}")
	if(${CMAKE_VERSION} VERSION_LESS "3.18.0")
		if(WIN32)
			message(FATAL_ERROR "CMake version 3.18.0 or higher is required to extract archives.")
		endif()
		execute_process(
			COMMAND mkdir ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem}
			COMMAND tar xvzf ${CMAKE_CURRENT_BINARY_DIR}/${archive_name} -C ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem}
			WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
	else()
		file(ARCHIVE_EXTRACT INPUT ${CMAKE_CURRENT_BINARY_DIR}/${archive_name} DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem})
	endif()
endif()

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name} AND EXISTS ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem})
	file(RENAME ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem}/webrtc-native-build-${webrtc_native_build_version}
		${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name})
	file(REMOVE_RECURSE ${CMAKE_CURRENT_BINARY_DIR}/${archive_stem})
endif()

set(boringssl_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")
set(webrtc_native_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")
set(libyuv_INCLUDE ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")

if(WIN32)
	set(boringssl_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/boringssl.lib CACHE INTERNAL "")
	set(webrtc_native_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/webrtc.lib CACHE INTERNAL "")
	set(libyuv_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libyuv_internal.lib CACHE INTERNAL "")
else()
	# TODO: this is a patch because these two thin archives do not link on MacOS 13 in CI.
	if(APPLE)
		file(GLOB boringssl_object_files ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/boringssl/*.o)
		set(boringssl_LIBRARY ${boringssl_object_files} CACHE INTERNAL "")

		file(GLOB libyuv_object_files ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libyuv_internal/*.o)
		set(libyuv_LIBRARY ${libyuv_object_files} CACHE INTERNAL "")
	else()
		set(boringssl_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libboringssl.a CACHE INTERNAL "")
		set(libyuv_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libyuv_internal.a CACHE INTERNAL "")
	endif()

	if(APPLE)
		set(webrtc_native_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libwebrtc.a "-framework Foundation" "-framework AVFoundation" "-framework CoreAudio" "-framework AudioToolbox" "-framework CoreGraphics" CACHE INTERNAL "")
	else()
		set(webrtc_native_LIBRARY ${CMAKE_CURRENT_BINARY_DIR}/${extracted_directory_name}/lib/libwebrtc.a dl X11 expat CACHE INTERNAL "")
	endif()
endif()
