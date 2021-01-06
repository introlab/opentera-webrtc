import SignalingClient from './SignalingClient';


class StreamClient extends SignalingClient {
  constructor(signalingServerConfiguration, streamConfiguration, rtcConfiguration, logger) {
    super(signalingServerConfiguration, logger);

    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }
    if (!window.MediaStream) {
      throw new Error('MediaStream is not supported.');
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

  _createRtcPeerConnection() {
    let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

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
        this._logger('RtcPeerConnection ontrack event, event=', event);

        if (!(id in this._remoteStreams)) {
          this._remoteStreams[id] = new window.MediaStream();
          this._remoteStreams[id].addTrack(event.track, this._remoteStreams[id]);
          this._onAddRemoteStream(id, this.getClientName(id), this.getClientData(id), this._remoteStreams[id]);
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
