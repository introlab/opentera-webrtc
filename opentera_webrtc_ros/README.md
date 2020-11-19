# Installation

The following ROS packages are required:
* roscpp
* cv_bridge
* std_msgs
* sensor_msgs

Additionally, some environment variables must be configured to link against the OpenTera WebRTC native client:
* `OPEN_TERA_WEBRTC_NATIVE_CLIENT_ROOT_DIR` :
points to `OpenteraWebrtcNativeClient` directory.
* `OPEN_TERA_WEBRTC_NATIVE_CLIENT_LIB` :
points to the instance of `libOpenteraWebrtcNativeClient.a` file to link against.

Finally the OpenTera WebRTC native client and its dependencies must have been built with same build type, Debug or
Release as the desired build output.

# RosTopicStreamer

## Description

Implement a ROS node `RosTopicStreamer` that subscribe to a `sensor_msgs::Image` topic and streams it to a WebRTC call.
The node accepts all incoming calls and advertise its video stream.

### Subscribes

* image_raw : `sensor_msgs::Image`

## Default Parameters

```yaml
~stream:
  is_screen_cast: false     # Is the image source a screen capture?
  needs_denoising: false    # Does the image source needs denoising?

~signaling:
  server_url: "http://localhost:8080" # Signaling server URL
  client_name: "streamer" # Peer name as which to join the room
  room_name: "chat"       # Room name to join
  room_password: "abc"    # Room password
```

# RosDataChannelBridge

## Description

Implement a ROS node that publish received messages on the WebRTC
data channel. It also forwards messages received on the WebRTC data channel to ROS.

### Subscribes

* ros_data : `std_msgs::String`

### Advertises

* webrtc_data : `std_msgs::String`

## Default Parameters

```yaml
~signaling:
  server_url: "http://localhost:8080" # Signaling server URL
  client_name: "data_bridge" # Peer name as which to join the room
  room_name: "chat"       # Room name to join
  room_password: "abc"    # Room password
```
