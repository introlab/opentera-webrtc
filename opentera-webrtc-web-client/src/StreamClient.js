import SignallingClient from './SignallingClient';


class StreamClient {
  constructor(signallingServerConfiguration, streamConfiguration, rtcConfiguration) {
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

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._streamConfiguration = streamConfiguration;
    this._rtcConfiguration = rtcConfiguration;

    this._signallingClient = null;
    this._rtcPeerConnections = {};
    this._remoteStreams = {};
    
    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onSignallingConnectionError = () => {};

    this._onRoomClientsChanged = () => {};
    
    this._onAddRemoteStream = () => {};
    this._onClientConnect = () => {};
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

  _getRtcPeerConnection(id) {
    if (id in this._rtcPeerConnections) {
      return this._rtcPeerConnections[id];
    }
    else {
      let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);
      
      this._connectRtcPeerConnectionEvents(id, rtcPeerConnection);
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
  
  _connectRtcPeerConnectionEvents(id, rtcPeerConnection) {
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
    rtcPeerConnection.onconnectionstatechange = event => {
      switch(rtcPeerConnection.connectionState) {
      case 'connected':
        this._onClientConnect(id, this._signallingClient.getClientName(id), event);
        break;

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

  _disconnectRtcPeerConnectionEvents(rtcPeerConnection) {
    rtcPeerConnection.ontrack = () => {};
    rtcPeerConnection.onconnectionstatechange = () => {};
  }

  _removeConnection(id) {
    if (id in this._remoteStreams) {
      this._remoteStreams[id].getTracks().forEach(track => track.stop());
      delete this._remoteStreams[id];
    }

    if (id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      delete this._rtcPeerConnections[id];
    }
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
      this._disconnectRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
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
    this._closeAllRemoteStreams();
    this._closeAllRtcPeerConnections();
    this._signallingClient.updateRoomClients();
  }

  close () {
    this._closeAllRemoteStreams();

    if (this._signallingClient !== null) {
      let signallingClient = this._signallingClient;
      this._signallingClient = null;
      signallingClient.close();
    }
    
    this._closeAllRtcPeerConnections();
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
  
  set onAddRemoteStream(onAddRemoteStream) {
    this._onAddRemoteStream = onAddRemoteStream;
  }

  set onClientConnect(onClientConnect) {
    this._onClientConnect = onClientConnect;
  }

  set onClientDisconnect(onClientDisconnect) {
    this._onClientDisconnect = onClientDisconnect;
  }
}

export default StreamClient;
