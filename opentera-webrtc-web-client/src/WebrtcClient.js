import WebSocketSignalingClient from './Signaling/WebSocketSignalingClient';

function isPromise(obj) {
  return obj && typeof obj.then === 'function' && Object.prototype.toString.call(obj) === '[object Promise]';
}

/**
 * @brief Represents the base class of DataChannelClient, StreamClient and StreamDataChannelClient.
 */
class WebrtcClient {
  /**
   * @brief Creates a signaling client with the specified configurations.
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
   * @param {CallableFunction} logger An optional logger callback
   */
  constructor(signalingServerConfiguration, logger) {
    if (this.constructor === WebrtcClient) {
      throw new TypeError('Abstract class "WebrtcClient" cannot be instantiated directly.');
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

    this._logger = logger;

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
    this._onClientConnectionFail = () => {};

    this._offerOptions = {};

    this._signalingClient = new WebSocketSignalingClient(signalingServerConfiguration, logger);
    this._connectSignalingClientCallbacks();
  }

  _connectSignalingClientCallbacks() {
    this._signalingClient.onSignalingConnectionOpen = () => { this._onSignalingConnectionOpen(); };
    this._signalingClient.onSignalingConnectionClose = () => { this.close(); };
    this._signalingClient.onSignalingConnectionError = error => {
      this.close();
      this._onSignalingConnectionError(error);
    };

    this._signalingClient.onRoomClientsChange = (clients) => {
      this._logger('SignalingServer room-clients event, clients=', clients);

      this._clients = clients;
      this._updateClientNamesById(clients);
      this._updateClientDatumById(clients);
      this._onRoomClientsChange(this._addConnectionStateToClients(this._clients));
    };

    this._signalingClient.makePeerCall = ids => { this._makePeerCall(ids); };
    this._signalingClient.peerCallReceived = data => { this._peerCallReceived(data); };
    this._signalingClient.peerCallAnswerReceived = data => { this._peerCallAnswerReceived(data); };
    this._signalingClient.iceCandidateReceived = data => { this._addIceCandidate(data); };

    this._signalingClient.closeAllPeerConnections = () => {
      this._logger('SignalingServer close-all-peer-connections-request-received event');
      this.hangUpAll();
    };
  }

  /**
   * @brief Connects the client the signaling server.
   * @returns {Promise<void>}
   */
  async connect() {
    this._logger('WebrtcClient.connect method call');
    await this._signalingClient.connect();
  }

  _disconnect() {
    this._signalingClient.disconnect();

    this._getAllRtcPeerConnection().forEach(c => this._disconnectRtcPeerConnectionEvents(c));

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

      this._signalingClient.makePeerCallAnswer(data.fromId, answer);
    }
    else {
      this._signalingClient.rejectCall(data.fromId);
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

    ids = ids.filter(id => id != this.id);
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

        this._signalingClient.callPeer(id, offer);
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
      this._signalingClient.sendIceCandidate(id, event.candidate);
    };

    rtcPeerConnection.onconnectionstatechange = () => {
      switch(rtcPeerConnection.connectionState) {
      case 'connected':
        this._logger('RtcPeerConnection connected event id=', id);

        this._onClientConnect(id, this.getClientName(id), this.getClientData(id));
        break;

      case 'failed':
        this._logger('RtcPeerConnection failed event id=', id);
        this._removeConnection(id, false);
        this._onClientConnectionFail(id, this.getClientName(id), this.getClientData(id));
        this.updateRoomClients();
        break;

      case 'disconnected':
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

  _removeConnection(id, callOnClientDisconnect) {
    callOnClientDisconnect = typeof callOnClientDisconnect !== 'undefined' ? callOnClientDisconnect : true;

    if (id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      delete this._rtcPeerConnections[id];
      this._alreadyAcceptedCalls = this._alreadyAcceptedCalls.filter(x => x != id);

      if (callOnClientDisconnect) {
        this._onClientDisconnect(id, this.getClientName(id), this.getClientData(id));
      }
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
    if (this._signalingClient.isConnected) {
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
    this._logger('WebrtcClient.callAll method call');

    this._alreadyAcceptedCalls = this._clients.map(client => client.id);
    this._signalingClient.callAll();
  }

  /**
   * @brief Calls the specified clients.
   * @param {Array<String>} ids The client ids to call
   */
  callIds(ids) {
    this._logger('WebrtcClient.callIds method call, ids=', ids);

    this._alreadyAcceptedCalls = ids;
    this._signalingClient.callIds(ids);
  }

  /**
   * @brief Hangs up all clients.
   */
  hangUpAll() {
    this._logger('WebrtcClient.hangUpAll method call');

    this._closeAllRtcPeerConnections();
    this.updateRoomClients();
  }

  /**
   * @brief Closes all room peer connections.
   */
  closeAllRoomPeerConnections() {
    this._logger('WebrtcClient.closeAllRoomPeerConnections method call');

    this._signalingClient.closeAllRoomPeerConnections();
  }

  /**
   * @brief Gets the client name from the client id.
   * @param {String} id The client id
   * @returns {String} The client name
   */
  getClientName(id) {
    return this._clientNamesById[id];
  }

  /**
   * @brief Gets the client data from the client id.
   * @param {String} id The client id
   * @returns {Object} The client name
   */
  getClientData(id) {
    return this._clientDatumById[id];
  }

  /**
   * @brief Indicates if the client is connected to the signaling server.
   * @return {Boolean} true if the client is connected to the signaling server
   */
  get isConnected() {
    return this._signalingClient.isConnected;
  }

  /**
   * @brief Indicates if the client is connected to at least one client (RTCPeerConnection).
   * @return {Boolean} true if the client is connected to at least one client (RTCPeerConnection)
   */
  get isRtcConnected() {
    return Object.keys(this._rtcPeerConnections).length > 0;
  }

  /**
   * @brief Returns the client id.
   * @return {String} The client id
   */
  get id() {
    return this._signalingClient.sessionId;
  }

  /**
   * @brief Returns the connected room client ids.
   * @return {Array<String>} The connected room client ids
   */
  get connectedRoomClientIds() {
    return Object.keys(this._rtcPeerConnections);
  }

  /**
   * @brief Returns the room clients.
   * @returns {Array<Object>} An array of client objects
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
   * @param {CallableFunction} onSignalingConnectionOpen The callback
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
   * @param {CallableFunction} onSignalingConnectionClose The callback
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
   * @param {CallableFunction} onSignalingConnectionError The callback
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
   * @param {CallableFunction} onRoomClientsChange The callback
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
   * @param {CallableFunction} callAcceptor The callback
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
   * @param {CallableFunction} onCallReject The callback
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
   * @param {CallableFunction} onClientConnect The callback
   */
  set onClientConnect(onClientConnect) {
    this._onClientConnect = onClientConnect;
  }

  /**
   * @brief Sets the callback that is called when a client peer connection closes.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param {CallableFunction} onClientDisconnect The callback
   */
  set onClientDisconnect(onClientDisconnect) {
    this._onClientDisconnect = onClientDisconnect;
  }

  /**
   * @brief Sets the callback that is called when a client peer connection fails.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param {CallableFunction} onClientConnectionFail The callback
   */
  set onClientConnectionFail(onClientConnectionFail) {
    this._onClientConnectionFail = onClientConnectionFail;
  }
}

export default WebrtcClient;
