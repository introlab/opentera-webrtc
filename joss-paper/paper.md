---
title: 'OpenTera WebRTC: A User Friendly WebRTC Wrapper for JavaScript, C++ and Python'
tags:
  - WebRTC
  - C++
  - Python
  - JavaScript
authors:
  - name: Marc-Antoine Maheux
    orcid: 0000-0002-3983-8754
    affiliation: 1
  - name: Dominic Létourneau
    orcid: 0000-0001-7825-7533
    affiliation: 1
  - name: Philippe Warren
    orcid: 0009-0008-4466-0963
    affiliation: 1
  - name: François Michaud
    orcid: 0000-0002-3639-7770
    affiliation: 1
affiliations:
  - name: Interdisciplinary Institute for Technological Innovation (3IT), Université de Sherbrooke, Canada
    index: 1
date: 15 March 2023
bibliography: paper.bib
---

# Summary
The COVID-19 pandemic changed the landscape of telehealth and videoconferencing practices, having people of all ages and fields become users of virtual meetings and even social activities [@saltzman_loneliness_2020].
Conventional videoconferencing solutions basically establish an audio and video link between two or more participants.
However, advanced solutions like telehealth systems may require interfacing remote biometric sensors and devices (e.g., vital sign monitoring devices, pan–tilt–zoom (PTZ) cameras, and telepresence mobile robots [@kristoffersson_review_2013]), streaming of multiple video feeds, and providing specialized graphical interfaces to improve situation awareness and reduce cognitive load of the users [@panchea_opentera_2022].
Simple audio-video links also reveal insufficient to address the specific needs of older adults [@cortellessa_ai_2021], for instance in terms of easy installation and ease of use, functionalities to address loneliness [@hung_facilitators_2022] or to implement socially distanced games [@ramirez_gomez_socially_2022; @yuan_tabletop_2021].
With the objective of providing a more generic telecommunication framework, we have designed OpenTera [@opentera; @panchea_opentera_2022], a micro-service architecture for audio-video-data telecommunication services and telehealth management.

WebRTC^[[https://webrtc.org/](https://webrtc.org/)] is the de facto technology to develop videoconference platforms because it is secured and built for web browsers, hence improving accessibility compared to having to install a software application locally.
This makes WebRTC a good choice to develop OpenTera.
A JavaScript API is available in modern web browsers, but the developer needs to create a web server to initialize the peer-to-peer calls, which is usually named a signaling server.
The library used by most web browsers to include WebRTC support is the WebRTC Native library^[[https://webrtc.github.io/webrtc-org/native-code/](https://webrtc.github.io/webrtc-org/native-code/)].
However, adding sensors to web-based videoconference setups is limited by their compatibility with web browsers, bounded by their APIs and sandbox configuration.
Typically, web browsers support mouse, keyboard, gamepad, webcam and microphone devices. However, experimental APIs are available to use Bluetooth^[[https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API](https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API)], serial^[[https://wicg.github.io/serial/](https://wicg.github.io/serial/)] and USB^[[https://developer.mozilla.org/en-US/docs/Web/API/USB](https://developer.mozilla.org/en-US/docs/Web/API/USB)] devices, but the devices must be compatible and these APIs are not supported on all browsers.
Also, in the context of telepresence robots, a web interface built with WebRTC can control a robot, but the robot needs to run the equivalent of a web browser for the JavaScript API.
This is often done via the integration of Chromium web engine (Electron, Qt) and requires code in JavaScript, which might not be optimized.

To address these limitations, this paper presents the OpenTera WebRTC project, a multiplatform library to develop web and native applications for customizable videoconferencing, telehealth and telepresence robotics applications. 
It provides bidirectional peer-to-peer multipoint communication of audio streams, multi-video feed streaming and data, and a signaling server hosted on secure, local servers, independent of tier actors and online services.

# Project Description
The OpenTera WebRTC library provides simple and similar APIs in JavaScript for web applications and in C++ and Python for native applications.
Developing native applications makes it possible to integrate a variety of devices and reduce resource usage.
Also, the library includes a secured signaling server to reduce the development time when using the library.
The C++ API uses the WebRTC Native library and is packaged to be easy to build.
Video and audio streams are provided to the library as raw images and raw audio frames to increase flexibility.
The Python API is done with pybind11 [@pybind11] to create Python bindings of the C++ API.
The Python API is easier to use, but the C++ API is faster.
To develop telepresence robots, the C++ API is integrated with ROS [@quigley_ros_2009] because this middleware is widely adopted by the research community.

GStreamer is adapted to the WebRTC Native library to provide hardware acceleration to encode and decode video streams.
So, the native APIs are well suited for embedded applications.
\autoref{tab:hardware-acceleration-compatibility} presents the hardware acceleration compatibility at the present time.
More codecs and hardware configurations can easily be added in the future.



Table: Hardware Acceleration Compatibility \label{tab:hardware-acceleration-compatibility}

| Operating System | Hardware                           | VP8                   | VP9      | H.264                 |
|------------------|------------------------------------|-----------------------|----------|-----------------------|
| Linux            | Jetson TX2/Nano                    | Encoding and Decoding | Decoding | Encoding and Decoding |
| Linux            | Jetson Xavier NX                   |                       | Decoding | Encoding and Decoding |
| Linux            | Jetson AGX Xavier                  |                       | Decoding | Encoding and Decoding |
| Linux            | Jetson Orin / Orin Nano / Orin NX  |                       | Decoding | Encoding and Decoding |
| Linux            | Raspberry Pi 4                     |                       |          | Encoding and Decoding |
| Linux            | Intel, AMD, NVIDIA with the VA-API | Encoding and Decoding | Decoding | Encoding and Decoding |
| MacOS            | Mac with the Apple Media API       |                       |          | Encoding and Decoding |


# Statement of need
WebRTC Native can be used to create native applications directly, but the library is hard to build and it is designed to use webcams and integrated peripherals.
OpenTera WebRTC native APIs add the capacity to provide any audio and video streams, making it more generic and flexible.
The integration of BoringSSL and Socket.IO code removes the need to wrap and understand other communication libraries.
Therefore, the users of OpenTera WebRTC can connect their applications to the signaling server easily.

\autoref{tab:webrtc-library-summary} presents a summary of the available WebRTC libraries to create native applications.
WebRTC ROS [@webrtc_ros] and MixedReality-WebRTC [@mixedreality-webrtc] also wrap the WebRTC Native library, but the versions used are pretty old and neither are no longer actively maintained.
GStreamer [@gstreamer] has a plugin to perform WebRTC calls. However, it is hard to use and the plugin is currently available in the gst-bad plugins, which means it is probably incomplete and misses important features to comply with the standard.
The aiortc library [@aiortc] and WebRTC.rs [@webrtc-rs] replicate the web WebRTC API in Python and Rust, respectively.
The former uses third-party party libraries for the audio and video codecs, and the latter does not yet have any codec support.
Nevertheless, WebRTC.rs supports streaming of already encoded audio and video streams.

Overall, OpenTera WebRTC provides interesting capabilities compared to the other WebRTC libraries because it offers hardware acceleration for embedded platforms, has a unified API across all languages, integrates the signaling server and wraps an up-to-date WebRTC Native version, which can be updated easily by our automated build on GitHub.


Table: WebRTC Library Summary \label{tab:webrtc-library-summary}

| Library             | Web Browser API | Native API  | Raw Audio Streams | Raw Video Streams | Hardware Acceleration | Signaling Server |
|---------------------|-----------------|-------------|-------------------|-------------------|-----------------------|------------------|
| WebRTC ROS          | X               | ROS         |                   | X                 |                       | Integrated       |
| GStreamer           |                 | C, Python   | X                 | X                 | X                     | Example          |
| aiortc              |                 | Python      | X                 | X                 |                       | Example          |
| MixedReality-WebRTC |                 | C++, C#     |                   | X                 |                       | Example          |
| WebRTC.rs           |                 | Rust        |                   |                   |                       | Example          |
| OpenTera WebRTC     | X               | C++, Python | X                 | X                 | X                     | Integrated       |


# Acknowledgements
This work was supported by the Natural Sciences and Engineering Research Council of Canada (NSERC), the Fonds de recherche du Québec – Nature et technologies (FRQNT) and the Network of Centres of Excellence of Canada on Aging Gracefully across Environments using Technology to Support Wellness, Engagement, and Long Life (AGE-WELL).

# References
