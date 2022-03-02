set(webrtc_native_build_version "4389.e7d9f7.130" CACHE INTERNAL "")

if(WIN32)
	message(FATAL_ERROR "Windows is not supported yet.")
elseif(UNIX)
	if(APPLE)
		set(architecture "osx64")
		# set(archive_name webrtc-native-build-macos-10.15-${architecture}-${CMAKE_BUILD_TYPE}-${webrtc_native_build_version}.tar.gz)
		set(archive_name webrtc-native-build-macos-10.15-${architecture}-Release-${webrtc_native_build_version}.tar.gz)
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

		if (NOT ${LSB_RELEASE_VERSION_SHORT} MATCHES "18.04" AND NOT ${LSB_RELEASE_VERSION_SHORT} MATCHES "20.04")
			message(FATAL_ERROR "Ubuntu version not supported: ${LSB_RELEASE_VERSION_SHORT}, must be 18.04 or 20.04.")
		endif()

		# set(archive_name webrtc-native-build-${LSB_RELEASE_ID_SHORT}-${LSB_RELEASE_VERSION_SHORT}-${architecture}-${CMAKE_BUILD_TYPE}-${webrtc_native_build_version}.tar.gz)
		set(archive_name webrtc-native-build-${LSB_RELEASE_ID_SHORT}-${LSB_RELEASE_VERSION_SHORT}-${architecture}-Release-${webrtc_native_build_version}.tar.gz)
	endif(APPLE)

	set(webrtc_native_build_archive_url "https://github.com/introlab/webrtc-native-build/releases/download/${webrtc_native_build_version}/${archive_name}")

	if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name})
		message(STATUS "Downloading : ${webrtc_native_build_archive_url}")
		file(DOWNLOAD
			${webrtc_native_build_archive_url}
			${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}
			TIMEOUT 60 #seconds
			TLS_VERIFY ON
			SHOW_PROGRESS)
	endif()

	set(extracted_directory_name "webrtc-native-build-${webrtc_native_build_version}")

	if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name})
		message(STATUS "Extracting ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}")
		execute_process(
			COMMAND tar xvzf ${CMAKE_CURRENT_SOURCE_DIR}/${archive_name}
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
	endif()

	set(boringssl_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")
	set(boringssl_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/lib/libboringssl.a CACHE INTERNAL "")
	set(webrtc_native_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")

	if(APPLE)
		set(webrtc_native_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/lib/libwebrtc.a "-framework Foundation" "-framework AVFoundation" "-framework CoreAudio" "-framework AudioToolbox" "-framework CoreGraphics" CACHE INTERNAL "")
	else(APPLE)
      	set(webrtc_native_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/lib/libwebrtc.a dl X11 expat CACHE INTERNAL "")
	endif(APPLE)

	set(libyuv_INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/include CACHE INTERNAL "")
	set(libyuv_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/${extracted_directory_name}/lib/libyuv_internal.a CACHE INTERNAL "")
else()
    message(FATAL_ERROR "Unsupported operating system.")
endif()
