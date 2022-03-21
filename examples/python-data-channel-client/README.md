# python-data-channel-client
This example shows how to use the Python library to create a client that communicates with another one by a WebRTC data channel. This example should be used with [web-data-channel-client](../web-data-channel-client).

## How to use
```bash
cd ../..
mkdir build
cd build
cmake ..
cmake --build . --config Release|Debug
cmake --install .

cd ../examples/python-data-channel-client/
python3 python_data_channel_client.py
```
