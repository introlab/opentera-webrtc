# python-stream-client
This example shows how to use the Python library to create a client that sends and receives a video stream and an audio stream. This example should be used with [web-stream-client](../web-stream-client).

You need to compile the target "opentera_webrtc_native_client" of "opentera-webrtc-native-client" and copy the module file in this folder.

## How to use
```bash
cd ../../opentera-webrtc-native-client
mkdir build
cd build
cmake ..
make

cp OpenteraWebrtcNativeClient/python/bin/Release/opentera_webrtc_native_client.*.so ../examples/python-stream-client/
cd ../examples/python-stream-client/
python3 python_stream_client.py
```
