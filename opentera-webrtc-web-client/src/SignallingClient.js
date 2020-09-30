import io from 'socket.io-client';


class SignallingClient {
  constructor(signallingServerConfiguration, logger) {
    if (this.constructor === SignallingClient) {
      throw new TypeError('Abstract class "SignallingClient" cannot be instantiated directly.');
    }
    if (this._createRtcPeerConnection === undefined) {
      throw new TypeError('_createRtcPeerConnection is missing.');
    }

    if (!window.RTCSessionDescription) {
      throw new Error('RTCSessionDescription is not supported.');
    }
    if (!logger) {
      logger = () => {};
    }

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._logger = logger;

    this._socket = null;
    this._rtcPeerConnections = {};

    this._clients = [];
    this._clientNamesById = {};
    this._clientDatumById = {};

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onSignallingConnectionError = () => {};
    this._onRoomClientsChanged = () => {};

    this._onClientConnect = () => {};
    this._onClientDisconnect = () => {};
  }
  
  async connect() {
    this._logger('SignallingClient.connect method call');

    this._socket = io(this._signallingServerConfiguration.url);
    this._connectEvents();

    await new Promise((resolve, reject) => {
      this._socket.on('connect', () => {
        this._logger('SignallingServer connect event');

        resolve();
      });
      this._socket.on('connect_error', error => {
        this._logger('SignallingServer connect_error event');

        reject(error);
      });
      this._socket.on('connect_timeout', error => {
        this._logger('SignallingServer connect_timeout event');

        reject(error);
      });
    });
    
    let data = {
      name: this._signallingServerConfiguration.name,
      data: this._signallingServerConfiguration.data,
      room: this._signallingServerConfiguration.room,
      password: this._signallingServerConfiguration.password
    };
    this._socket.emit('join-room', data, isJoined => {
      this._logger('SignallingServer join-room event, isJoined=', isJoined);
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
    this._socket.on('disconnect', () => {
      this._logger('SignallingServer disconnect event');

      this._disconnect();
    });

    this._socket.on('room-clients', clients => {
      this._logger('SignallingServer room-clients event, clients=', clients);

      this._clients = clients;
      this._updateClientNamesById(clients);
      this._updateClientDatumById(clients);
      this._onRoomClientsChanged(this._addConnectionStateToClients(this._clients));
    });

    this._socket.on('make-peer-call', async ids => await this._makePeerCall(ids));
    this._socket.on('peer-call-received', async data => await this._peerCallReceived(data));
    this._socket.on('peer-call-answer-received', async data => await this._peerCallAnswerReceived(data));
    this._socket.on('close-all-peer-connections-request-received', () => {
      this._logger('SignallingServer close-all-peer-connections-request-received event');

      this.hangUpAll();
    });

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

    this._getAllRtcPeerConnection().forEach(c => this._disconnectRtcPeerConnectionEvents(c));
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
    this._logger('SignallingServer peer-call-received event, data=', data);

    let rtcPeerConnection = this._createRtcPeerConnection(data.fromId, false);
    this._rtcPeerConnections[data.fromId] = rtcPeerConnection;
    this._connectRtcPeerConnectionEvents(data.fromId, rtcPeerConnection);

    await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.offer));
    
    let answer = await rtcPeerConnection.createAnswer();    
    await rtcPeerConnection.setLocalDescription(new window.RTCSessionDescription(answer));

    data = { toId: data.fromId, answer: answer };
    this._socket.emit('make-peer-call-answer', data);
  }

  async _peerCallAnswerReceived(data) {
    this._logger('SignallingServer peer-call-answer-received event, data=', data);

    let rtcPeerConnection = this._rtcPeerConnections[data.fromId];
    await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.answer));
  }

  async _addIceCandidate(data) {
    this._logger('SignallingServer ice-candidate-received event, data=', data);

    if (data && data.candidate) {
      this._rtcPeerConnections[data.fromId].addIceCandidate(data.candidate);
    }
  }

  async _makePeerCall(ids) {
    this._logger('SignallingServer make-peer-call event, ids=', ids);

    ids = ids.filter(id => id != this._socket.id);
    ids.forEach(async id => {
      if (this._hasRtcPeerConnection(id)) {
        return;
      }

      let rtcPeerConnection = this._createRtcPeerConnection(id, true);
      this._rtcPeerConnections[id] = rtcPeerConnection;
      this._connectRtcPeerConnectionEvents(id, rtcPeerConnection);

      let offer = await rtcPeerConnection.createOffer();
      await rtcPeerConnection.setLocalDescription(new RTCSessionDescription(offer));

      let data = { toId: id, offer: offer };
      this._socket.emit('call-peer', data);
    });
  }

  _connectRtcPeerConnectionEvents(id, rtcPeerConnection) {
    rtcPeerConnection.onicecandidate = event => {
      let data = { toId: id, candidate: event.candidate };
      this._socket.emit('send-ice-candidate', data);
    };

    rtcPeerConnection.onconnectionstatechange = () => {
      switch(rtcPeerConnection.connectionState) {
      case 'connected':
        this._logger('RtcPeerConnection connected event id=', id);

        this._onClientConnect(id, this.getClientName(id), this.getClientData(id));
        break;

      case 'disconnected':
      case 'failed':
      case 'closed':
        this._logger('RtcPeerConnection disconnected, failed or closed event, id=', id);

        this._removeConnection(id);
        this.updateRoomClients();
        break;
      }
    };
  }

  _disconnectRtcPeerConnectionEvents(rtcPeerConnection) {
    rtcPeerConnection.onicecandidate = () => {};
    rtcPeerConnection.onconnectionstatechange = () => {};
  }

  _removeConnection(id) {
    if (id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      delete this._rtcPeerConnections[id];
      this._onClientDisconnect(id, this.getClientName(id), this.getClientData(id));
    }
  }

  _addConnectionStateToClients(clients) {
    let newClients = [];
    clients.forEach(client => {
      newClients.push({
        id: client.id,
        name: client.name,
        data: client.data,
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

  _updateClientDatumById(clients) {
    this._clientDatumById = {};
    clients.forEach(client => {
      this._clientDatumById[client.id] = client.data;
    });
  }

  _hasRtcPeerConnection(id) {
    return id in this._rtcPeerConnections;
  }

  _getAllRtcPeerConnection() {
    return Object.values(this._rtcPeerConnections);
  }

  _closeAllRtcPeerConnections() {
    for (let id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      this._onClientDisconnect(id, this.getClientName(id), this.getClientData(id));
      delete this._rtcPeerConnections[id];
    }
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
    this._logger('SignallingClient.callAll method call');

    this._socket.emit('call-all');
  }

  callIds(ids) {
    this._logger('SignallingClient.callIds method call, ids=', ids);

    this._socket.emit('call-ids', ids);
  }

  hangUpAll() {
    this._logger('SignallingClient.hangUpAll method call');

    this._closeAllRtcPeerConnections();
    this.updateRoomClients();
  }

  closeAllRoomPeerConnections() {
    this._logger('SignallingClient.closeAllRoomPeerConnections method call');

    this._socket.emit('close-all-room-peer-connections');
  }

  getClientName(id) {
    return this._clientNamesById[id];
  }

  getClientData(id) {
    return this._clientDatumById[id];
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

  set onClientConnect(onClientConnect) {
    this._onClientConnect = onClientConnect;
  }

  set onClientDisconnect(onClientDisconnect) {
    this._onClientDisconnect = onClientDisconnect;
  }
}

export default SignallingClient;
