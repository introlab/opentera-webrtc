# cpp-camera-stream-client

This example shows how to use the C++ library to create a client that sends and receives a video stream and an audio stream. 
The video source is the webcam, the audio source is the default capture device and the audio output is the default playback device. 
This example should be used with [web-stream-client](../web-stream-client).

## How to use

```bash
cd ../..
mkdir build
cd build
cmake ..
cmake --build . --config Release|Debug

cd bin/Release
./CppCameraStreamClient
```
