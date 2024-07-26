# OpenTera - WebRTC

[![Actions Status](https://github.com/introlab/opentera-webrtc/workflows/cpp-python-tests/badge.svg)](https://github.com/introlab/opentera-webrtc/actions)

[WebRTC](https://webrtc.org/) is a standard for real-time audio/video/data communication and is mostly used in web browsers.
For the [OpenTera](https://github.com/introlab/opentera) micro-service architecture, we wanted to use Google's native WebRTC implementation for robots so we would have more control on the streams and develop a framework that is optimized for robot teleoperation. We also wanted to use hardware acceleration if possible on embedded platforms.

This project provides:

* [A signaling server](signaling-server) written in Python that can be used as a stand-alone server or embedded in our [OpenTera Teleoperation Micro-Service](https://github.com/introlab/opentera-teleop-service)
* [A native libwebrtc wrapper](opentera-webrtc-native-client/OpenteraWebrtcNativeClient/src) that is used as a C++ base library for clients using our signaling server.
* [GStreamer video codec factories for libwebrtc](opentera-webrtc-native-client/OpenTeraWebrtcNativeGstreamer) to enable both software and [hardware encoders/decoders](#hardware-acceleration-with-gstreamer) if available.
* Possiblity to force usage of a particular codec with [VideoStreamConfiguration](opentera-webrtc-native-client/OpenteraWebrtcNativeClient/include/OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h)
* [Python 3 bindings](opentera-webrtc-native-client/OpenteraWebrtcNativeClient/python) that are used as a Python module for clients using our signaling server.
* [A JavaScript library](opentera-webrtc-web-client) That is used for web client developement and teleoperation interface using our signaling server.

## License

By default, libwebrtc is built with non-free codecs. See [webrtc-native-build](https://github.com/introlab/webrtc-native-build#license) to build without them.

* [Apache License, Version 2.0](LICENSE)
* For GStreamer components, please read [this](opentera-webrtc-native-client/OpenteraWebrtcNativeGStreamer/README.md).


## API Documentation

API Documentation is available [here](https://introlab.github.io/opentera-webrtc).

## How to Contribute

Please read the [Code of Conduct](CODE_OF_CONDUCT.md) and [CONTRIBUTING](CONTRIBUTING.md) files.

## Dependencies

* Most dependencies are part of the project with git submodules in the [opentera-webrtc-native-client/3rdParty](opentera-webrtc-native-client/3rdParty) directory.

## Build Instructions

* Please see [Build Instructions](BUILD_INSTRUCTIONS.md).

## Examples

### C++

* [data-channel-client](examples/cpp-data-channel-client)
* [cpp-video-stream-client](examples/cpp-video-stream-client)
* [cpp-camera-stream-client](examples/cpp-camera-stream-client)

### Python

* [data-channel-client](examples/python-data-channel-client)
* [stream-client](examples/python-stream-client)

### Javascript

* [data-channel-client](examples/web-data-channel-client)
* [stream-client](examples/web-stream-client)
* [stream-data-channel-client](examples/web-stream-data-channel-client)

## Hardware Acceleration with GStreamer

Hardware acceleration depends on the platform you are using to build opentera-webrtc. The following table summarizes supported hardware and GStreamer plugins:

| Platform                          | VP8                         | VP9           | H.264                        |
|-----------------------------------|-----------------------------|---------------|------------------------------|
| Jetson TX2/Nano                   | nvv4l2vp8enc, nvv4l2decoder | nvv4l2decoder | nvv4l2h264enc, nvv4l2decoder |
| Jetson Xavier NX                  | X                           | nvv4l2decoder | nvv4l2h264enc, nvv4l2decoder |
| Jetson AGX Xavier                 | X                           | nvv4l2decoder | nvv4l2h264enc, nvv4l2decoder |
| Jetson Orin / Orin Nano / Orin NX | X                           | nvv4l2decoder | nvv4l2h264enc, nvv4l2decoder |
| Raspberry Pi 4                    | X                           | X             | v4l2h264enc, v4l2h264dec     |
| VA-API                            | vaapivp8enc, vaapivp8dec    | vaapivp9dec   | vaapih264enc, vaapih264dec   |
| Apple Media                       | X                           | X             | vtenc_h264, vtdec            |

Note that VP9 encoding is under development, see issue [117](https://github.com/introlab/opentera-webrtc/issues/117).

## ROS

* The C++ library and Python bindings are used in our [opentera_webrtc_ros ROS package](https://github.com/introlab/opentera-webrtc-ros). RGB images published in ROS topics can be easily used for streaming in a RTCPeerConnection using our signaling server. Hardware acceleration and codec selection can be used if available.

## Frontend

* The JavaScript library is used for our [opentera WebRTC teleoperation frontend](https://github.com/introlab/opentera-webrtc-teleop-frontend).

## Authors

* Marc-Antoine Maheux (@mamaheux)
* Dominic Létourneau (@doumdi)
* Philippe Warren (@philippewarren)
* François Michaud (@michaudf)

## Contributors

* Cédric Godin (@godced)
* Ian-Mathieu Joly (@joli-1801)

## Sponsor

![IntRoLab](images/IntRoLab.png)

[IntRoLab - Intelligent / Interactive / Integrated / Interdisciplinary Robot Lab](https://introlab.3it.usherbrooke.ca)
