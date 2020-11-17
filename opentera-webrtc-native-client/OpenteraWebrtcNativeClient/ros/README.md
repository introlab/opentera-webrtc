# Description

Implement a ROS Node `RosTopicStreamer` that subscribe to a `sensor_msgs::Image` topic and streams it to a WebRTC call.
The node accepts all incoming calls and advertise its video stream.

# Default Parameters

```yaml
~stream:
  topic: "camera/image_raw" # Image topic to stream
  is_screen_cast: false     # Is the image source a screen capture?
  needs_denoising: false    # Does the image source needs denoising?

~signaling:
  server_url: "http://localhost:8080" # Signaling server URL
  client_name: "streamer" # Peer name as which to join the room
  room_name: "chat"       # Room name to join
  room_password: "abc"    # Room password
```