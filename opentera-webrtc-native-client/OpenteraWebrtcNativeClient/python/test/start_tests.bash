#!/bin/bash

# Copy library
cp ../../../build/OpenteraWebrtcNativeClient/python/bin/Release/opentera_webrtc_native_client.cpython-38-x86_64-linux-gnu.so .

# Discover tests and run them
python3 -m unittest discover . "*_test.py"

