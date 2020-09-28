import SignallingClient from './SignallingClient';


class StreamDataChannelClient {
  constructor(signallingServerConfiguration, streamConfiguration, dataChannelConfiguration, rtcConfiguration) {
    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }

    if (!signallingServerConfiguration) {
      throw new Error('signallingServerConfiguration is required');
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

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._streamConfiguration = streamConfiguration;
    this._dataChannelConfiguration = dataChannelConfiguration;
    this._rtcConfiguration = rtcConfiguration;

    this._signallingClient = null;
    this._rtcPeerConnections = {};
    this._dataChannels = {};
    this._remoteStreams = {};

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onSignallingConnectionError = () => {};

    this._onRoomClientsChanged = () => {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (id, event) => {
      console.error('Data channel error: ' + id + ', ' + event);
    };

    this._onAddRemoteStream = () => {};
    this._onClientDisconnect = () => {};
  }

  async connect() {
    let hasRtcPeerConnection = id => this._hasRtcPeerConnection(id);
    let getRtcPeerConnection = (id, isCaller) => this._getRtcPeerConnection(id, isCaller);
    let getAllRtcPeerConnection = () => this._getAllRtcPeerConnection();
    this._signallingClient = new SignallingClient(this._signallingServerConfiguration,
      hasRtcPeerConnection, getRtcPeerConnection, getAllRtcPeerConnection);

    this._signallingClient.onConnectionOpen = this._onSignallingConnectionOpen;
    this._signallingClient.onConnectionClose = () => {
      this.close();
      this._onSignallingConnectionClose();
    };
    this._signallingClient.onConnectionError = error => {
      this.close();
      this._onSignallingConnectionError(error);
    };
    this._signallingClient.onRoomClientsChanged = this._onRoomClientsChanged;

    await this._signallingClient.connect();
  }

  _hasRtcPeerConnection(id) {
    return id in this._rtcPeerConnections;
  }

  _getRtcPeerConnection(id, isCaller) {
    if (id in this._rtcPeerConnections) {
      return this._rtcPeerConnections[id];
    }
    else {
      let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

      if (isCaller) {
        let dataChannel = rtcPeerConnection.createDataChannel(this._signallingServerConfiguration.room,
          this._dataChannelConfiguration);
        this._dataChannels[id] = dataChannel;
        this._connectDataChannelEvents(id, dataChannel);
      }

      this._connectRtcPeerConnectionEvents(id, rtcPeerConnection, isCaller);
      if (this._streamConfiguration.localStream) {
        this._streamConfiguration.localStream.getTracks().forEach(track => {
          rtcPeerConnection.addTrack(track, this._streamConfiguration.localStream);
        });      
      }

      this._rtcPeerConnections[id] = rtcPeerConnection;
      return rtcPeerConnection;
    }
  } 
  
  _getAllRtcPeerConnection() {
    return Object.values(this._rtcPeerConnections);
  }

  _connectRtcPeerConnectionEvents(id, rtcPeerConnection, isCaller) {
    if (!isCaller) {
      rtcPeerConnection.ondatachannel = ({ channel: dataChannel }) => {
        this._dataChannels[id] = dataChannel;
        this._connectDataChannelEvents(id, dataChannel);
      };
    }

    if (!this._streamConfiguration.isSendOnly) {
      rtcPeerConnection.ontrack = event => {
        if (!(id in this._remoteStreams)) {
          this._remoteStreams[id] = new window.MediaStream();
          this._remoteStreams[id].addTrack(event.track, this._remoteStreams[id]);
          this._onAddRemoteStream(id, this._signallingClient.getClientName(id), this._remoteStreams[id]);
          this._signallingClient.updateRoomClients();
        }
        else {
          this._remoteStreams[id].addTrack(event.track, this._remoteStreams[id]);
        }
      };
    }
    rtcPeerConnection.onconnectionstatechange = () => {
      switch(rtcPeerConnection.connectionState) {
      case 'disconnected':
      case 'failed':
      case 'closed':
        this._removeConnection(id);
        this._onClientDisconnect(id, this._signallingClient.getClientName(id), event);
        this._signallingClient.updateRoomClients();
        break;
      }
    };
  }

  _disconnectDataChannelsRtcPeerConnectionEvents(rtcPeerConnection) {
    rtcPeerConnection.ondatachannel = () => {};

    rtcPeerConnection.ontrack = () => {};
    rtcPeerConnection.onconnectionstatechange = () => {};
  }

  _connectDataChannelEvents(id, dataChannel) {
    dataChannel.onmessage = event => {
      this._onDataChannelMessage(id, this._signallingClient.getClientName(id), event.data);
    };
    dataChannel.onopen = event => {
      this._onDataChannelOpen(id, this._signallingClient.getClientName(id), event);
      this._signallingClient.updateRoomClients();
    };
    dataChannel.onclose = event => {
      this._removeConnection(id);
      this._onDataChannelClose(id, this._signallingClient.getClientName(id), event);
      this._signallingClient.updateRoomClients();
    };
    dataChannel.onerror = event => {
      this._removeConnection(id);
      this._onDataChannelError(id, this._signallingClient.getClientName(id), event);
      this._onDataChannelClose(id, this._signallingClient.getClientName(id), event);
      this._signallingClient.updateRoomClients();
    };
  }

  _disconnectDataChannelEvents(dataChannel) {
    dataChannel.onmessage = () => {};
    dataChannel.onopen = () => {};
    dataChannel.onclose = () => {};
    dataChannel.onerror = () => {};
  }

  _removeConnection(id) {
    if (id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      delete this._dataChannels[id];
    }

    if (id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
      delete this._remoteStreams[id];
    }

    if (id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      delete this._rtcPeerConnections[id];
    }
  }

  _closeAllDataChannels() {
    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
    }
    this._dataChannels = {};
  }

  _closeAllRemoteStreams() {
    for (let id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
    }
    this._remoteStreams = {};
  }

  _closeAllRtcPeerConnections() {
    for (let id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
    }
    this._rtcPeerConnections = {};
  }

  callAll() {
    this._signallingClient.callAll();
  }

  callIds(ids) {
    this._signallingClient.callIds(ids);
  }

  hangUpAll() {
    this._signallingClient.hangUpAll();
    this._closeAllDataChannels();
    this._closeAllRemoteStreams();
    this._closeAllRtcPeerConnections();
    this._signallingClient.updateRoomClients();
  }

  close () {
    this._closeAllDataChannels();
    this._closeAllRemoteStreams();

    if (this._signallingClient !== null) {
      let signallingClient = this._signallingClient;
      this._signallingClient = null;
      signallingClient.close();
    }
    
    this._closeAllRtcPeerConnections();
  }

  sendTo(data, ids) {
    ids.forEach(id => this._dataChannels[id].send(data));
  }

  sendToAll(data) {
    for (let id in this._dataChannels) {
      this._dataChannels[id].send(data);
    }
  }

  get isConnected() {
    return this._signallingClient !== null;
  }

  get isRtcConnected() {
    return Object.keys(this._rtcPeerConnections).length > 0;
  }

  get id() {
    if (this._signallingClient !== null) {
      return this._signallingClient.id;
    }
    else {
      return null;
    }
  }

  get connectedRoomClientIds() {
    return Object.keys(this._rtcPeerConnections);
  }

  get roomClients() {
    return this._signallingClient !== null ? this._signallingClient.clients : [];
  }

  set onSignallingConnectionOpen(onSignallingConnectionOpen) {
    this._onSignallingConnectionOpen = onSignallingConnectionOpen;
  }

  set onSignallingConnectionClose(onSignallingConnectionClose) {
    this._onSignallingConnectionClose = onSignallingConnectionClose;
  }

  set onSignallingConnectionError(onSignallingConnectionError) {
    this._onSignallingConnectionError = onSignallingConnectionError;
  }

  set onRoomClientsChanged(onRoomClientsChanged) {
    this._onRoomClientsChanged = onRoomClientsChanged;
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

  set onClientDisconnect(onClientDisconnect) {
    this._onClientDisconnect = onClientDisconnect;
  }
}

export default StreamDataChannelClient;
