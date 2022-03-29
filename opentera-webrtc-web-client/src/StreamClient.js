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

    this._offerOptions = {
      offerToReceiveAudio: true,
      offerToReceiveVideo: true
    };

    this._isLocalAudioMuted = false;
    this._isLocalVideoMuted = false;
  }

  _createRtcPeerConnection() {
    let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

    if (this._streamConfiguration.localStream) {
      this._streamConfiguration.localStream.getTracks().forEach(track => {
        if (track.kind == 'audio') {
          track.enabled = !this._isLocalAudioMuted;
        }
        else if (track.kind == 'video') {
          track.enabled = !this._isLocalVideoMuted;
        }
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

  close() {
    this._closeAllRemoteStreams();
    super.close();
  }

  get isLocalAudioMuted() {
    return this._isLocalAudioMuted;
  }

  get isLocalVideoMuted() {
    return this._isLocalVideoMuted;
  }

  muteLocalAudio() {
    this.setLocalAudioMuted(true);
  }

  unmuteLocalAudio() {
    this.setLocalAudioMuted(false);
  }

  setLocalAudioMuted(muted) {
    this._isLocalAudioMuted = muted;
    this._setAllLocalTracksEnabled('audio', !muted);
  }

  muteLocalVideo() {
    this.setLocalVideoMuted(true);
  }

  unmuteLocalVideo() {
    this.setLocalVideoMuted(false);
  }

  setLocalVideoMuted(muted) {
    this._isLocalVideoMuted = muted;
    this._setAllLocalTracksEnabled('video', !muted);
  }

  _setAllLocalTracksEnabled(kind, enabled) {
    this._getAllRtcPeerConnection().forEach(rtcPeerConnection => {
      let senders = rtcPeerConnection.getSenders();
      senders.forEach(sender => {
        if (sender.track.kind == kind) {
          sender.track.enabled = enabled;
        }
      });
    });
  }

  set onAddRemoteStream(onAddRemoteStream) {
    this._onAddRemoteStream = onAddRemoteStream;
  }
}

export default StreamClient;
