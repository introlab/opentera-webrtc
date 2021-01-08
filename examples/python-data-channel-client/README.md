# python-data-channel-client
This example shows how to use the Python library to create a client that communicates with another one by a WebRTC data channel. This example should be used with [web-data-channel-client](../web-data-channel-client).

You need to compile the target "opentera_webrtc_native_client" of "opentera-webrtc-native-client" and copy the module file in this folder.

## How to use
```bash
cd ../../opentera-webrtc-native-client
mkdir build
cd build
cmake ..
make

cp OpenteraWebrtcNativeClient/python/bin/Release/opentera_webrtc_native_client.*.so ../examples/python-data-channel-client/
cd ../examples/python-data-channel-client/
python3 python_data_channel_client.py
```
