import io from 'socket.io-client';


class SignallingClient {
  constructor(signallingServerConfiguration) {
    if (this.constructor === SignallingClient) {
      throw new TypeError('Abstract class "SignallingClient" cannot be instantiated directly.');
    }
    if (this._createRtcPeerConnection === undefined) {
      throw new TypeError('_createRtcPeerConnection is missing.');
    }
    if (this._removeConnection === undefined) {
      throw new TypeError('_removeConnection is missing.');
    }

    if (!window.RTCSessionDescription) {
      throw new Error('RTCSessionDescription is not supported.');
    }

    this._signallingServerConfiguration = signallingServerConfiguration;

    this._socket = null;
    this._rtcPeerConnections = {};

    this._clients = [];
    this._clientNamesById = {};

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onSignallingConnectionError = () => {};
    this._onRoomClientsChanged = () => {};
  }
  
  async connect() {
    this._socket = io(this._signallingServerConfiguration.url);
    this._connectEvents();

    await new Promise((resolve, reject) => {
      this._socket.on('connect', () => resolve());
      this._socket.on('connect_error', error => reject(error));
      this._socket.on('connect_timeout', error => reject(error));
    });
    
    let data = {
      name: this._signallingServerConfiguration.name,
      room: this._signallingServerConfiguration.room,
      password: this._signallingServerConfiguration.password
    };
    this._socket.emit('join-room', data, isJoined => {
      if (isJoined) {
        this._onSignallingConnectionOpen();
      }
      else {
        this.close();
        this._onSignallingConnectionError('Invalid password');
      }
    });
  }

  _connectEvents() {
    this._socket.on('disconnect', () => this._disconnect());

    this._socket.on('room-clients', clients => {
      this._clients = clients;
      this._updateClientNamesById(clients);
      this._onRoomClientsChanged(this._addConnectionStateToClients(this._clients));
    });

    this._socket.on('make-peer-call', async ids => await this._makePeerCall(ids));
    this._socket.on('peer-call-received', async data => await this._peerCallReceived(data));
    this._socket.on('peer-call-answer-received', async data => await this._peerCallAnswerReceived(data));

    this._socket.on('ice-candidate-received', async data => await this._addIceCandidate(data));
  }

  _disconnectEvents() {
    this._socket.off('connect');
    this._socket.off('connect_error');
    this._socket.off('connect_timeout');
    this._socket.off('disconnect');

    this._socket.off('room-clients');

    this._socket.off('make-peer-call');
    this._socket.off('peer-call-received');
    this._socket.off('peer-call-answer-received');

    this._getAllRtcPeerConnection().forEach(c => c.onicecandidate = () => {});
    this._socket.off('ice-candidate');
  }

  _disconnect() {
    this._disconnectEvents();
    this._socket.close();
    this._socket = null;
    this._clients = [];
    this.close();
    this._onSignallingConnectionClose();
  }

  async _peerCallReceived(data) {
    let rtcPeerConnection = this._createRtcPeerConnection(data.fromId, false);
    this._rtcPeerConnections[data.fromId] = rtcPeerConnection;
    this._connectOnIceCandidateEvent(data.fromId, rtcPeerConnection);

    await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.offer));
    
    let answer = await rtcPeerConnection.createAnswer();    
    await rtcPeerConnection.setLocalDescription(new window.RTCSessionDescription(answer));

    data = { toId: data.fromId, answer: answer };
    this._socket.emit('make-peer-call-answer', data);
  }

  async _peerCallAnswerReceived(data) {
    let rtcPeerConnection = this._rtcPeerConnections[data.fromId];
    await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.answer));
  }

  async _addIceCandidate(data) {
    if (data && data.candidate) {
      this._rtcPeerConnections[data.fromId].addIceCandidate(data.candidate);
    }
  }

  async _makePeerCall(ids) {
    ids = ids.filter(id => id != this._socket.id);
    ids.forEach(async id => {
      if (this._hasRtcPeerConnection(id)) {
        return;
      }

      let rtcPeerConnection = this._createRtcPeerConnection(id, true);
      this._rtcPeerConnections[id] = rtcPeerConnection;
      this._connectOnIceCandidateEvent(id, rtcPeerConnection);

      let offer = await rtcPeerConnection.createOffer();
      await rtcPeerConnection.setLocalDescription(new RTCSessionDescription(offer));

      let data = { toId: id, offer: offer };
      this._socket.emit('call-peer', data);
    });
  }

  _connectOnIceCandidateEvent(id, rtcPeerConnection) {
    rtcPeerConnection.onicecandidate = event => {
      let data = { toId: id, candidate: event.candidate };
      this._socket.emit('send-ice-candidate', data);
    };
  }

  _addConnectionStateToClients(clients) {
    let newClients = [];
    clients.forEach(client => {
      newClients.push({
        id: client.id,
        name: client.name,
        isConnected: this._hasRtcPeerConnection(client.id) || client.id == this.id
      });
    });

    return newClients;
  }

  _updateClientNamesById(clients) {
    this._clientNamesById = {};
    clients.forEach(client => {
      this._clientNamesById[client.id] = client.name;
    });
  }

  _hasRtcPeerConnection(id) {
    return id in this._rtcPeerConnections;
  }

  _getAllRtcPeerConnection() {
    return Object.values(this._rtcPeerConnections);
  }

  _closeAllRtcPeerConnections() {
    this._getAllRtcPeerConnection().forEach(rtcPeerConnection => {
      rtcPeerConnection.close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(rtcPeerConnection);
    });
    this._rtcPeerConnections = {};
  }

  updateRoomClients() {
    this._onRoomClientsChanged(this._addConnectionStateToClients(this._clients));
  }

  close() {
    if (this._socket !== null) {
      this._disconnect();
    }
    // The RTC peer connections need to close after the socket because of socket can create a RTC connection after RTC peer connections closing.
    this._closeAllRtcPeerConnections();
  }

  callAll() {
    this._socket.emit('call-all');
  }

  callIds(ids) {
    this._socket.emit('call-ids', ids);
  }

  hangUpAll() {
    this._getAllRtcPeerConnection().forEach(c => c.onicecandidate = () => {});

    this._closeAllRtcPeerConnections();
    this.updateRoomClients();
  }

  getClientName(id) {
    return this._clientNamesById[id];
  }

  get isConnected() {
    return this._socket !== null;
  }

  get isRtcConnected() {
    return Object.keys(this._rtcPeerConnections).length > 0;
  }

  get id() {
    if (this._socket !== null) {
      return this._socket.id;
    }
    else {
      return null;
    }
  }

  get connectedRoomClientIds() {
    return Object.keys(this._rtcPeerConnections);
  }

  get roomClients() {
    return this._addConnectionStateToClients(this._clients);
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
}

export default SignallingClient;
