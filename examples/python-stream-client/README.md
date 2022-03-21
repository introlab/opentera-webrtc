# python-stream-client
This example shows how to use the Python library to create a client that sends and receives a video stream and an audio stream. This example should be used with [web-stream-client](../web-stream-client).

## How to use
```bash
cd ../..
mkdir build
cd build
cmake ..
cmake --build . --config Release|Debug
cmake --install .

cd ../examples/python-stream-client/
python3 -m pip install -r requirements.txt
python3 python_stream_client.py
```
