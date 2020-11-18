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