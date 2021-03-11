#!/bin/bash

sysroot=$1

echo "--------------------------- $sysroot"

a=arm64
extras=""

common_flags="is_clang=false is_component_build=false treat_warnings_as_errors=false fatal_linker_warnings=false use_gio=false use_rtti=true use_custom_libcxx=false use_custom_libcxx_for_host=false rtc_enable_protobuf=false rtc_include_tests=false rtc_use_h264=true proprietary_codecs=true target_os=\"linux\" target_cpu=\"$a\"$extras enable_iterator_debugging=false rtc_build_examples=false rtc_use_pipewire=false use_sysroot=true sysroot=\"$sysroot\""


gn gen out/Debug --args="is_debug=true $common_flags"
gn gen out/Release --args="is_debug=false $common_flags"
