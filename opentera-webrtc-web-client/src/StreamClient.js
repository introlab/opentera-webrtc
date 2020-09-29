import SignallingClient from './SignallingClient';


class StreamClient extends SignallingClient {
  constructor(signallingServerConfiguration, streamConfiguration, rtcConfiguration) {
    super(signallingServerConfiguration);

    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }
    if (!window.MediaStream) {
      throw new Error('MediaStream is not supported.');
    }

    if (!signallingServerConfiguration) {
      throw new Error('signallingServerConfiguration is required');
    }
    if (!streamConfiguration) {
      throw new Error('streamConfiguration is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._streamConfiguration = streamConfiguration;
    this._rtcConfiguration = rtcConfiguration;

    this._rtcPeerConnections = {};
    this._remoteStreams = {};
    
    this._onAddRemoteStream = () => {};
  }

  _createRtcPeerConnection(id) {
    let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

    this._connectRtcPeerConnectionEvents(id, rtcPeerConnection);
    if (this._streamConfiguration.localStream) {
      this._streamConfiguration.localStream.getTracks().forEach(track => {
        rtcPeerConnection.addTrack(track, this._streamConfiguration.localStream);
      });
    }

    return rtcPeerConnection;
  }
  
  _connectRtcPeerConnectionEvents(id, rtcPeerConnection) {
    super._connectRtcPeerConnectionEvents(id, rtcPeerConnection);

    if (!this._streamConfiguration.isSendOnly) {
      rtcPeerConnection.ontrack = event => {
        if (!(id in this._remoteStreams)) {
          this._remoteStreams[id] = new window.MediaStream();
          this._remoteStreams[id].addTrack(event.track, this._remoteStreams[id]);
          this._onAddRemoteStream(id, this.getClientName(id), this._remoteStreams[id]);
          this.updateRoomClients();
        }
        else {
          this._remoteStreams[id].addTrack(event.track, this._remoteStreams[id]);
        }
      };
    }
  }

  _disconnectRtcPeerConnectionEvents(rtcPeerConnection) {
    super._disconnectRtcPeerConnectionEvents(rtcPeerConnection);

    rtcPeerConnection.ontrack = () => {};
  }

  _removeConnection(id) {
    super._removeConnection(id);

    if (id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
      delete this._remoteStreams[id];
    }
  }

  _closeAllRemoteStreams() {
    for (let id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
    }
    this._remoteStreams = {};
  }

  hangUpAll() {
    super.hangUpAll();
    this._closeAllRemoteStreams();
  }

  close () {
    this._closeAllRemoteStreams();
    super.close();
  }
  
  set onAddRemoteStream(onAddRemoteStream) {
    this._onAddRemoteStream = onAddRemoteStream;
  }
}

export default StreamClient;
