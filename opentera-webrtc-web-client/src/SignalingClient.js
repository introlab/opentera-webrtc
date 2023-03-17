import io from 'socket.io-client';

const SignalingProtocolVersion = 1;

function isPromise(obj) {
  return obj && typeof obj.then === 'function' && Object.prototype.toString.call(obj) === '[object Promise]';
}

/**
 * @brief Represents the base class of DataChannelClient, StreamClient and StreamDataChannelClient.
 */
class SignalingClient {
  /**
   * @brief Creates a signaling client with the specified configurations.
   *
   * @param signalingServerConfiguration The signaling server configuration
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
   * @param logger An optional logger callback
   */
  constructor(signalingServerConfiguration, logger) {
    if (this.constructor === SignalingClient) {
      throw new TypeError('Abstract class "SignalingClient" cannot be instantiated directly.');
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

    this._signalingServerConfiguration = signalingServerConfiguration;
    this._logger = logger;

    this._socket = null;
    this._rtcPeerConnections = {};

    this._clients = [];
    this._clientNamesById = {};
    this._clientDatumById = {};

    this._alreadyAcceptedCalls = [];

    this._onSignalingConnectionOpen = () => {};
    this._onSignalingConnectionClose = () => {};
    this._onSignalingConnectionError = () => {};
    this._onRoomClientsChange = () => {};

    this._callAcceptor = () => { return true; };
    this._onCallReject = () => {};

    this._onClientConnect = () => {};
    this._onClientDisconnect = () => {};

    this._offerOptions = {};
  }

  /**
   * @brief Connects the client the signaling server.
   * @returns {Promise<void>}
   */
  async connect() {
    this._logger('SignalingClient.connect method call');

    let url = new URL(this._signalingServerConfiguration.url);
    let path = url.pathname;
    url = url.protocol + '//' + url.hostname + ':' + url.port;
    if (path === '/') {
      path = '/socket.io';
    }

    this._socket = io(url, {path: path});
    this._connectEvents();

    await new Promise((resolve, reject) => {
      this._socket.on('connect', () => {
        this._logger('SignalingServer connect event');

        resolve();
      });
      this._socket.on('connect_error', error => {
        this._logger('SignalingServer connect_error event');

        reject(error);
      });
      this._socket.on('connect_timeout', error => {
        this._logger('SignalingServer connect_timeout event');

        reject(error);
      });
    });
    
    let data = {
      name: this._signalingServerConfiguration.name,
      data: this._signalingServerConfiguration.data,
      room: this._signalingServerConfiguration.room,
      password: this._signalingServerConfiguration.password,
      protocolVersion: SignalingProtocolVersion
    };
    this._socket.emit('join-room', data, isJoined => {
      this._logger('SignalingServer join-room event, isJoined=', isJoined);
      if (isJoined) {
        this._onSignalingConnectionOpen();
      }
      else {
        this.close();
        this._onSignalingConnectionError('Invalid password or invalid protocol version');
      }
    });
  }

  _connectEvents() {
    this._socket.on('disconnect', () => {
      this._logger('SignalingServer disconnect event');

      this._disconnect();
    });

    this._socket.on('room-clients', clients => {
      this._logger('SignalingServer room-clients event, clients=', clients);

      this._clients = clients;
      this._updateClientNamesById(clients);
      this._updateClientDatumById(clients);
      this._onRoomClientsChange(this._addConnectionStateToClients(this._clients));
    });

    this._socket.on('make-peer-call', async ids => await this._makePeerCall(ids));
    this._socket.on('peer-call-received', async data => await this._peerCallReceived(data));
    this._socket.on('peer-call-answer-received', async data => await this._peerCallAnswerReceived(data));
    this._socket.on('close-all-peer-connections-request-received', () => {
      this._logger('SignalingServer close-all-peer-connections-request-received event');

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
    this._alreadyAcceptedCalls = [];
    this.close();
    this._onSignalingConnectionClose();
  }

  async _peerCallReceived(data) {
    this._logger('SignalingServer peer-call-received event, data=', data);

    if (await this._getCallAcceptance(data.fromId)) {
      let rtcPeerConnection = this._createRtcPeerConnection(data.fromId, false);
      this._rtcPeerConnections[data.fromId] = rtcPeerConnection;
      this._connectRtcPeerConnectionEvents(data.fromId, rtcPeerConnection);

      await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.offer));

      let answer = await rtcPeerConnection.createAnswer();
      await rtcPeerConnection.setLocalDescription(new window.RTCSessionDescription(answer));

      data = { toId: data.fromId, answer: answer };
      this._socket.emit('make-peer-call-answer', data);
    }
    else {
      data = { toId: data.fromId };
      this._socket.emit('make-peer-call-answer', data);
    }
  }

  async _peerCallAnswerReceived(data) {
    this._logger('SignalingServer peer-call-answer-received event, data=', data);

    if ('answer' in data) {
      let rtcPeerConnection = this._rtcPeerConnections[data.fromId];
      await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.answer));
    }
    else {
      this._onCallReject(data.fromId, this.getClientName(data.fromId), this.getClientData(data.fromId));
      this._removeConnection(data.fromId);
    }
  }

  async _addIceCandidate(data) {
    this._logger('SignalingServer ice-candidate-received event, data=', data);

    if (data && data.candidate && data.fromId in this._rtcPeerConnections) {
      this._rtcPeerConnections[data.fromId].addIceCandidate(data.candidate);
    }
  }

  async _makePeerCall(ids) {
    this._logger('SignalingServer make-peer-call event, ids=', ids);

    ids = ids.filter(id => id != this._socket.id);
    ids.forEach(async id => {
      if (this._hasRtcPeerConnection(id)) {
        return;
      }

      if (await this._getCallAcceptance(id)) {
        let rtcPeerConnection = this._createRtcPeerConnection(id, true);
        this._rtcPeerConnections[id] = rtcPeerConnection;
        this._connectRtcPeerConnectionEvents(id, rtcPeerConnection);

        let offer = await rtcPeerConnection.createOffer(this._offerOptions);
        await rtcPeerConnection.setLocalDescription(new RTCSessionDescription(offer));

        let data = { toId: id, offer: offer };
        this._socket.emit('call-peer', data);
      }
      else {
        this._onCallReject(id, this.getClientName(id), this.getClientData(id));
      }
    });
  }

  async _getCallAcceptance(id) {
    if (this._alreadyAcceptedCalls.includes(id)) {
      return true;
    }

    let response = this._callAcceptor(id, this.getClientName(id), this.getClientData(id));
    if (isPromise(response)) {
      response = await response;
    }
    return response;
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
      this._alreadyAcceptedCalls = this._alreadyAcceptedCalls.filter(x => x != id);
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
    this._alreadyAcceptedCalls = [];
  }

  updateRoomClients() {
    this._onRoomClientsChange(this._addConnectionStateToClients(this._clients));
  }

  /**
   * @brief Closes all client connections
   */
  close() {
    if (this._socket !== null) {
      this._disconnect();
    }
    // The RTC peer connections need to close after the socket because of socket can create a RTC connection after
    // RTC peer connections closing.
    this._closeAllRtcPeerConnections();
  }

  /**
   * @brief Calls all room clients.
   */
  callAll() {
    this._logger('SignalingClient.callAll method call');

    this._alreadyAcceptedCalls = this._clients.map(client => client.id);
    this._socket.emit('call-all');
  }

  /**
   * @brief Calls the specified clients.
   * @param ids The client ids to call
   */
  callIds(ids) {
    this._logger('SignalingClient.callIds method call, ids=', ids);

    this._alreadyAcceptedCalls = ids;
    this._socket.emit('call-ids', ids);
  }

  /**
   * @brief Hangs up all clients.
   */
  hangUpAll() {
    this._logger('SignalingClient.hangUpAll method call');

    this._closeAllRtcPeerConnections();
    this.updateRoomClients();
  }

  /**
   * @brief Closes all room peer connections.
   */
  closeAllRoomPeerConnections() {
    this._logger('SignalingClient.closeAllRoomPeerConnections method call');

    this._socket.emit('close-all-room-peer-connections');
  }

  /**
   * @brief Gets the client name from the client id.
   * @param id The client id
   * @returns The client name
   */
  getClientName(id) {
    return this._clientNamesById[id];
  }

  /**
   * @brief Gets the client data from the client id.
   * @param id The client id
   * @returns The client name
   */
  getClientData(id) {
    return this._clientDatumById[id];
  }

  /**
   * @brief Indicates if the client is connected to the signaling server.
   * @return true if the client is connected to the signaling server
   */
  get isConnected() {
    return this._socket !== null;
  }

  /**
   * @brief Indicates if the client is connected to at least one client (RTCPeerConnection).
   * @return true if the client is connected to at least one client (RTCPeerConnection)
   */
  get isRtcConnected() {
    return Object.keys(this._rtcPeerConnections).length > 0;
  }

  /**
   * @brief Returns the client id.
   * @return The client id
   */
  get id() {
    if (this._socket !== null) {
      return this._socket.id;
    }
    else {
      return null;
    }
  }

  /**
   * @brief Returns the connected room client ids.
   * @return The connected room client ids
   */
  get connectedRoomClientIds() {
    return Object.keys(this._rtcPeerConnections);
  }

  /**
   * @brief Returns the room clients.
   * @returns An array of client objects
   */
  get roomClients() {
    return this._addConnectionStateToClients(this._clients);
  }

  /**
   * @brief Sets the callback that is called when the signaling connection opens.
   *
   * @parblock
   * Callback parameter: None
   * @endparblock
   *
   * @param onSignalingConnectionOpen The callback
   */
  set onSignalingConnectionOpen(onSignalingConnectionOpen) {
    this._onSignalingConnectionOpen = onSignalingConnectionOpen;
  }

  /**
   * @brief Sets the callback that is called when the signaling connection closes.
   *
   * @parblock
   * Callback parameter: None
   * @endparblock
   *
   * @param onSignalingConnectionClose The callback
   */
  set onSignalingConnectionClose(onSignalingConnectionClose) {
    this._onSignalingConnectionClose = onSignalingConnectionClose;
  }

  /**
   * @brief Sets the callback that is called when a signaling connection error occurs.
   *
   * @parblock
   * Callback parameter:
   *  - error: The error message
   * @endparblock
   *
   * @param onSignalingConnectionError The callback
   */
  set onSignalingConnectionError(onSignalingConnectionError) {
    this._onSignalingConnectionError = onSignalingConnectionError;
  }

  /**
   * @brief Sets the callback that is called when the room client changes.
   *
   * @parblock
   * Callback parameter:
   *  - clients: The room clients
   * @endparblock
   *
   * @param onRoomClientsChange The callback
   */
  set onRoomClientsChange(onRoomClientsChange) {
    this._onRoomClientsChange = onRoomClientsChange;
  }

  /**
   * @brief Sets the callback that is used to accept or reject a call.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   *
   * Callback return value:
   *  - true to accept the call, false to reject the call
   * @endparblock
   *
   * @param callAcceptor The callback
   */
  set callAcceptor(callAcceptor) {
    this._callAcceptor = callAcceptor;
  }

  /**
   * @brief Sets the callback that is called when a call is rejected.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param onCallReject The callback
   */
  set onCallReject(onCallReject) {
    this._onCallReject = onCallReject;
  }

  /**
   * @brief Sets the callback that is called when a client peer connection opens.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param onClientConnect The callback
   */
  set onClientConnect(onClientConnect) {
    this._onClientConnect = onClientConnect;
  }

  /**
   * @brief Sets the callback that is called when an error occurs.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param onClientDisconnect The callback
   */
  set onClientDisconnect(onClientDisconnect) {
    this._onClientDisconnect = onClientDisconnect;
  }
}

export default SignalingClient;
