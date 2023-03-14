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
During the COVID-19 pandemic, people were not able to socialize in person due to lockdowns, curfews and social distancing measures.
So, their social activities migrate to online platforms [@saltzman_loneliness_2020], such as videoconference ones.
For example, people were remotely playing games with friends and family, but current technologies are not adapted to all types of games, such as tabletop games requiring non-verbal cues or containing a lot of physical components [@ramirez_gomez_socially_2022; @yuan_tabletop_2021].
Telepresence robots are another type of technology providing a way to stay in touch with loved ones.
Compared to videoconference platforms, the remote user is usually able to move in a remote environment and talk with a remote person [@kristoffersson_review_2013].
They are used in aged care settings to address loneliness [@hung_facilitators_2022], but ???.
Thus, technologies providing a way to stay in touch with people require future research to improve the platforms in contexts like remote gaming and aged care setting.
Doing so needs to develop new platforms because commercially available platforms are closed to modifications.

WebRTC is the de facto technology to develop videoconference platforms since it is secured and built for web browsers, hence accessible.
Video streams, audio streams and data can be exchanged between the participants of a call.
A JavaScript API is available in modern web browsers, but the developer needs to create a web server to initialize the peer-to-peer calls, which is usually named a signaling server.
The library used by most web browsers to include WebRTC support is the WebRTC Native library.
In the context of remote tabletop gaming, adding sensors to the videoconference setups to improve the gaming experience is limited by their compatibility with the web browsers.
On the other hand, a web interface built with WebRTC can control a telepresence robot, but the robot needs to run a web browser for the JavaScript API, which is not great in resource-constrained robots.

This paper presents OpenTera WebRTC which is a multiplatform library to develop web and native applications for videoconference and telepresence robots.
The library provides simple and similar APIs in JavaScript for web applications and in C++ and Python for native ones.
Also, the library includes a signaling server to reduce the development time when using the library.
The C++ API uses the WebRTC Native library and is packaged to be easy to build. Video and audio streams are provided to the library as raw images and raw audio frames to increase flexibility.
The Python API is done with pybind11 [@pybind11] to create Python bindings of the C++ API.
The Python API is easier to use, but the C++ API is faster.
To develop telepresence robots, the C++ API is integrated with ROS [@quigley_ros_2009] since it is a widely used middleware in robotic research.

GStreamer is addapted to the WebRTC Native library to provide hardware acceleration to encode and decode video streams.
So, the native APIs are well suited for embedded applications.
\autoref{tab:hardware-acceleration-compatibility} presents the hardware acceleration compatibility.



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
WebRTC Native can be used to create native applications, but the library is hard to build and it is designed to read the peripheral devices (microphones and webcams) easily.
So, wrapping WebRTC Native library to provide a simplified build and raw streams is useful.

\autoref{tab:webrtc-library-summary} present a summary of the available WebRTC libraries to create native applications
WebRTC ROS [@webrtc_ros] and MixedReality-WebRTC [@mixedreality-webrtc] also wrap the WebRTC Native library, but the version used is pretty old and neither are not actively maintained.
GStreamer [@gstreamer] has a plugin to perform WebRTC calls, but it is hard to use and the API is not complete.
The aiortc library [@aiortc] and WebRTC.rs [@webrtc-rs] reproduce the web WebRTC API in Python and Rust respectively. The former uses third-party party libraries for the audio and video codecs and the latter does not have any codec support, yet.
However, WebRTC.rs supports streaming of already encoded audio and video streams.

In the context of telepresence robot and videoconference research, OpenTera WebRTC is better than all presented libraries because it offers hardware acceleration for the embedded platform, has a unified API across all languages, wraps an up-to-date WebRTC Native version and integrates the signaling server.



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
