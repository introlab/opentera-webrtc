#!/bin/bash

a="$(dpkg-architecture -qDEB_BUILD_ARCH || uname -m)"
case "$a" in
	i?86)
		a=x86
		extras=""
	;;
	x86_64|amd64)
		a=x64
		extras=""
	;;
	armhf|armv*)
		a=arm
		extras=" arm_float_abi=\"hard\""
	;;
	arm64|aarch64)
		a=arm64
		extras=""
	;;
	*)
		echo>&2 "WARNING: Unknown target platform: $a, continuing anyway"
	;;
esac

common_flags="is_clang=false is_component_build=false treat_warnings_as_errors=false fatal_linker_warnings=false use_gio=false use_rtti=true use_custom_libcxx=false rtc_enable_protobuf=false rtc_include_tests=false rtc_use_h264=true use_sysroot=false proprietary_codecs=true host_cpu=\"$a\" current_cpu=\"$a\" target_cpu=\"$a\"$extras"
gn gen out/Debug --args="is_debug=true enable_dsyms=true symbol_level=2 $common_flags"
gn gen out/Release --args="is_debug=false enable_dsyms=false symbol_level=0 $common_flags"
