import WebrtcClient from './WebrtcClient';

/**
 * @brief A signaling client to join a WebRTC room and stream a video source.
 */
class StreamClient extends WebrtcClient {
  /**
   * @brief Creates a stream client
   *
   * @param {Object} signalingServerConfiguration The signaling server configuration
   * @code
   *  {
   *       url: 'signaling server URL',
   *       name: 'client name',
   *       data: {}, // Client custom data
   *       room: 'room name',
   *       password: 'password'
   *  }
   * @endcode
   *
   * @param {Object} streamConfiguration The stream configuration
   * @code
   *  {
   *       localStream: localVideo.srcObject, // Optional
   *       isSendOnly: false
   *  }
   * @endcode
   *
   * @param {Object} rtcConfiguration The RTC configuration
   *  See https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/RTCPeerConnection configuration parameter
   *
   * @param {CallableFunction} logger An optional logger callback
   */
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
    this._rtcConfiguration['sdpSemantics'] = 'unified-plan';

    this._rtcPeerConnections = {};
    this._remoteStreams = {};

    this._onAddRemoteStream = () => {};

    this._offerOptions = {
      offerToReceiveAudio: true,
      offerToReceiveVideo: true
    };

    this._isLocalAudioMuted = false;
    this._isRemoteAudioMuted = false;
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

  /**
   * @brief Hangs up all clients.
   */
  hangUpAll() {
    super.hangUpAll();
    this._closeAllRemoteStreams();
  }

  /**
   * @brief Closes all client connections
   */
  close() {
    this._closeAllRemoteStreams();
    super.close();
  }

  /**
   * @brief Indicates if the local audio is muted.
   * @return {Boolean} true if the local audio is muted.
   */
  get isLocalAudioMuted() {
    return this._isLocalAudioMuted;
  }

  /**
   * @brief Indicates if the remote audio is muted.
   * @return {Boolean} true if the remote audio is muted.
   */
  get isRemoteAudioMuted() {
    return this._isRemoteAudioMuted;
  }

  /**
   * @brief Indicates if the local video is muted.
   * @return {Boolean} true if the local audio is muted.
   */
  get isLocalVideoMuted() {
    return this._isLocalVideoMuted;
  }

  /**
   * @brief Mutes the local audio.
   */
  muteLocalAudio() {
    this.setLocalAudioMuted(true);
  }

  /**
   * @brief Unmutes the local audio.
   */
  unmuteLocalAudio() {
    this.setLocalAudioMuted(false);
  }

  /**
   * @brief Mutes or unmutes the local audio.
   * @param muted indicates if the local audio is muted or not (true or false)
   */
  setLocalAudioMuted(muted) {
    this._isLocalAudioMuted = muted;
    this._setAllLocalTracksEnabled('audio', !muted);
  }

  /**
   * @brief Mutes the remote audio.
   */
  muteRemoteAudio() {
    this.setRemoteAudioMuted(true);
  }

  /**
   * @brief Unmutes the remote audio.
   */
  unmuteRemoteAudio() {
    this.setRemoteAudioMuted(false);
  }

  /**
   * @brief Mutes or unmutes the remote audio.
   * @param {Boolean} muted indicates if the remote audio is muted or not
   */
  setRemoteAudioMuted(muted) {
    this._isRemoteAudioMuted = muted;
    this._setAllRemoteTracksEnabled('audio', !muted);
  }

  /**
   * @brief Mutes the local video.
   */
  muteLocalVideo() {
    this.setLocalVideoMuted(true);
  }

  /**
   * @brief Unmutes the local video.
   */
  unmuteLocalVideo() {
    this.setLocalVideoMuted(false);
  }

  /**
   * @brief Mutes or unmutes the local video.
   * @param {Boolean} muted indicates if the local video is muted or not
   */
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

  _setAllRemoteTracksEnabled(kind, enabled) {
    this._getAllRtcPeerConnection().forEach(rtcPeerConnection => {
      let receivers = rtcPeerConnection.getReceivers();
      receivers.forEach(receiver => {
        if (receiver.track.kind == kind) {
          receiver.track.enabled = enabled;
        }
      });
    });
  }

  /**
   * @brief Sets the callback that is called when a stream is added.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   *  - remoteStream: The remote stream
   * @endparblock
   *
   *
   * @param {CallableFunction} onAddRemoteStream The callback
   */
  set onAddRemoteStream(onAddRemoteStream) {
    this._onAddRemoteStream = onAddRemoteStream;
  }
}

export default StreamClient;
