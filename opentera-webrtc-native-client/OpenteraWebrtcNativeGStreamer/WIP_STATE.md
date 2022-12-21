# State of progress for the GStreamer implementation

This document tracks the current progress and trials of using a GStreamer pipeline to decode and encode video frames to benefit from hardware acceleration in OpenTera WebRTC.

## Current state

The current state of the implementation is that it is possible to create a GStreamer pipeline that can decode video frames.
The pipeline is created using the `parse_launch` function and syntax, but is is currently statically defined.
Commented lines allow to change the decoder element and codecs used.
Use the search function of your IDE to find the commented lines using the terms `vp8`, `vp9` or `h264`.

### Current support of elements and codecs
Only lists tested elements, all tested on Ubuntu 20.04 amd64.

- [x] **(software) `avdec_h264`**: Working!
- [x] **(software) `vp8dec`**: Working!
- [x] **(software) `vp9dec`**: Working, freezes after a while? (OpenCV issue?)
- [x] **(hardware/vaapi) `vaapih264dec`**: Working! (needs vaapipostproc)
- [x] **(hardware/vaapi) `vaapivp8dec`**: Working! (needs vaapipostproc)
- [x] **(hardware/vaapi) `vaapivp9dec`**: Working! (needs vaapipostproc)

## Tasks

- [x] Create RAII wrappers/helpers for GStreamer C types
- [x] Create conversion functions between webrtc and gstreamer frames
- [x] Create a simple working GStreamer decoder
- [ ] Create a simple working GStreamer encoder
- [x] (PARTIAL) Support building without GStreamer
- [x] Fallback to default webrtc software decoder/encoder or fail if no hardware dec/enc available
- [x] Add setting to disable fallback to default webrtc software decoder/encoder or fail if no hardware dec/enc available
- [x] Create a compounded decoder factory that instantiates an available decoder for the platform if it can
- [ ] Create a compounded encoder factory that instantiates an available encoder for the platform if it can
- [x] Refactor GStreamerVideoFrameLibWebRTC to reuse I420Buffers if possible instead of allocating anew for each frame (using a buffer pool of sorts)
- [ ] Refactor logging and verbosity (like current deleters for GStreamer types are all verbose)
- [ ] Lots of cleanup to do (comments and all)
- [x] Validate that the keyframes logic (waiting for keyframe) works or make it work in decoder
- [x] Handle multiple clients connected, and deconnection then reconnection of clients (shared gst::Gst and all)
- [x] Check if Python wrapper works with the GStreamer decoder/encoder (with .so/.dll/.dylib)
- [ ] Add tests for GStreamer decoder/encoder

## Codecs support

- [x] Initial simple test decoder support using software decoder in GStreamer
- [ ] Initial simple test encoder support using software encoder in GStreamer
- [x] Support VP8, VP9, H264, (H265?, AV1?) with hardware decoders (vaapi)
- [x] Support VP8, VP9, H264, (H265?, AV1?) with hardware decoders (raspberry pi)
- [x] Support VP8, VP9, H264, (H265?, AV1?) with hardware decoders (jetson xavier nx / jetson agx xavier / jetson orin agx)
- [ ] (MAYBE) Support VP8, VP9, H264, (H265?, AV1?) with hardware decoders (Windows platform API)
- [ ] (MAYBE) Support VP8, VP9, H264, (H265?, AV1?) with hardware decoders (MacOS platform API)
- [ ] Support VP8, VP9, H264, (H265?, AV1?) with hardware encoders (vaapi)
- [ ] Support VP8, VP9, H264, (H265?, AV1?) with hardware encoders (raspberry pi)
- [ ] Support VP8, VP9, H264, (H265?, AV1?) with hardware encoders (jetson xavier nx / jetson agx xavier / jetson orin agx)
- [ ] (MAYBE) Support VP8, VP9, H264, (H265?, AV1?) with hardware encoders (Windows platform API)
- [ ] (MAYBE) Support VP8, VP9, H264, (H265?, AV1?) with hardware encoders (MacOS platform API)
- [x] Support forcing a specific decoder/encoder type (specific codec)
- [ ] Support forcing a specific decoder/encoder element (using gstreamer element name)

Note that AV1 us currently supported by WebRTC, but H265 is not.

## Platforms support

- [ ] Support Linux (x86_64)
- [ ] Support Linux (arm64)
- [ ] (MAYBE) Support Windows (x86_64)
- [ ] (MAYBE) Support Windows (arm64)
- [ ] (MAYBE) Support macOS (x86_64)
- [ ] (MAYBE) Support macOS (arm64)
