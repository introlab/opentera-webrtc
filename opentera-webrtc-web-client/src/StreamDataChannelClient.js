import SignalingClient from './SignalingClient';


class StreamDataChannelClient extends SignalingClient {
  constructor(signalingServerConfiguration, streamConfiguration, dataChannelConfiguration, rtcConfiguration, logger) {
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
    if (!dataChannelConfiguration) {
      throw new Error('dataChannelConfiguration is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._streamConfiguration = streamConfiguration;
    this._dataChannelConfiguration = dataChannelConfiguration;
    this._rtcConfiguration = rtcConfiguration;
    this._rtcConfiguration['sdpSemantics'] = 'unified-plan';

    this._dataChannels = {};
    this._remoteStreams = {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (id, event) => {
      console.error('Data channel error: ' + id + ', ' + event);
    };

    this._onAddRemoteStream = () => {};

    this._offerOptions = {
      offerToReceiveAudio: true,
      offerToReceiveVideo: true
    };

    this._isLocalAudioMuted = false;
    this._isLocalVideoMuted = false;
  }

  _createRtcPeerConnection(id, isCaller) {
    let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

    if (isCaller) {
      let dataChannel = rtcPeerConnection.createDataChannel(this._signalingServerConfiguration.room,
        this._dataChannelConfiguration);
      this._dataChannels[id] = dataChannel;
      this._connectDataChannelEvents(id, dataChannel);
    }

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

    if (!(id in this._dataChannels)) {
      rtcPeerConnection.ondatachannel = ({ channel: dataChannel }) => {
        this._dataChannels[id] = dataChannel;
        this._connectDataChannelEvents(id, dataChannel);
      };
    }

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

    rtcPeerConnection.ondatachannel = () => {};
    rtcPeerConnection.ontrack = () => {};
  }

  _connectDataChannelEvents(id, dataChannel) {
    dataChannel.onmessage = event => {
      this._onDataChannelMessage(id, this.getClientName(id), this.getClientData(id), event.data);
    };
    dataChannel.onopen = () => {
      this._logger('onDataChannelOpen, id=', id);

      this._onDataChannelOpen(id, this.getClientName(id), this.getClientData(id));
      this.updateRoomClients();
    };
    dataChannel.onclose = () => {
      this._logger('onDataChannelClose, id=', id);

      this._removeConnection(id);
      this.updateRoomClients();
    };
    dataChannel.onerror = event => {
      this._logger('onDataChannelError, id=', id);

      this._removeConnection(id);
      this._onDataChannelError(id, this.getClientName(id), this.getClientData(id), event);
      this.updateRoomClients();
    };
  }

  _disconnectDataChannelEvents(dataChannel) {
    dataChannel.onmessage = () => {};
    dataChannel.onopen = () => {};
    dataChannel.onclose = () => {};
    dataChannel.onerror = () => {};
  }

  _removeConnection(id) {
    super._removeConnection(id);

    if (id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      this._onDataChannelClose(id, this.getClientName(id), this.getClientData(id));
      delete this._dataChannels[id];
    }

    if (id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
      delete this._remoteStreams[id];
    }
  }

  _closeAllDataChannels() {
    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      this._onDataChannelClose(id, this.getClientName(id), this.getClientData(id));
      delete this._dataChannels[id];
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
    this._closeAllDataChannels();
    this._closeAllRemoteStreams();
  }

  close () {
    this._closeAllDataChannels();
    this._closeAllRemoteStreams();

    super.close();
  }

  get isLocalAudioMuted() {
    return this._isLocalAudioMuted;
  }

  get isRemoteAudioMuted() {
    return this._isRemoteAudioMuted;
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

  muteRemoteAudio() {
    this.setRemoteAudioMuted(true);
  }

  unmuteRemoteAudio() {
    this.setRemoteAudioMuted(false);
  }

  setLocalRemoteMuted(muted) {
    this._isRemoteAudioMuted = muted;
    this._setAllRemoteTracksEnabled('audio', !muted);
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

  sendTo(data, ids) {
    ids.forEach(id => this._dataChannels[id].send(data));
  }

  sendToAll(data) {
    for (let id in this._dataChannels) {
      this._dataChannels[id].send(data);
    }
  }

  set onDataChannelMessage(onDataChannelMessage) {
    this._onDataChannelMessage = onDataChannelMessage;
  }

  set onDataChannelOpen(onDataChannelOpen) {
    this._onDataChannelOpen = onDataChannelOpen;
  }

  set onDataChannelClose(onDataChannelClose) {
    this._onDataChannelClose = onDataChannelClose;
  }

  set onDataChannelError(onDataChannelError) {
    this._onDataChannelError = onDataChannelError;
  }

  set onAddRemoteStream(onAddRemoteStream) {
    this._onAddRemoteStream = onAddRemoteStream;
  }
}

export default StreamDataChannelClient;
