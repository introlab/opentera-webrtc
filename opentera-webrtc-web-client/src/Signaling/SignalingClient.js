class SignalingClient {
  constructor(signalingServerConfiguration, logger) {
    if (this.constructor === SignalingClient) {
      throw new TypeError('Abstract class "SignalingClient" cannot be instantiated directly.');
    }

    this._signalingServerConfiguration = signalingServerConfiguration;
    this._logger = logger;

    this._onSignalingConnectionOpen = () => {};
    this._onSignalingConnectionClose = () => {};
    this._onSignalingConnectionError = () => {};

    this._onRoomClientsChange = () => {};

    this._makePeerCall = () => {};
    this._peerCallReceived = () => {};
    this._peerCallAnswerReceived = () => {};
    this._iceCandidateReceived = () => {};

    this._closeAllPeerConnections = () => {};
  }

  get room() {
    return this._signalingServerConfiguration.room;
  }

  set onSignalingConnectionOpen(onSignalingConnectionOpen) {
    this._onSignalingConnectionOpen = onSignalingConnectionOpen;
  }

  set onSignalingConnectionClose(onSignalingConnectionClose) {
    this._onSignalingConnectionClose = onSignalingConnectionClose;
  }

  set onSignalingConnectionError(onSignalingConnectionError) {
    this._onSignalingConnectionError = onSignalingConnectionError;
  }

  set onRoomClientsChange(onRoomClientsChange) {
    this._onRoomClientsChange = onRoomClientsChange;
  }

  set makePeerCall(makePeerCall) {
    this._makePeerCall = makePeerCall;
  }

  set peerCallReceived(peerCallReceived) {
    this._peerCallReceived = peerCallReceived;
  }

  set peerCallAnswerReceived(peerCallAnswerReceived) {
    this._peerCallAnswerReceived = peerCallAnswerReceived;
  }

  set iceCandidateReceived(iceCandidateReceived) {
    this._iceCandidateReceived = iceCandidateReceived;
  }

  set closeAllPeerConnections(closeAllPeerConnections) {
    this._closeAllPeerConnections = closeAllPeerConnections;
  }

  async connect() {
    throw new TypeError('The method "connect" is not overrided.');
  }

  disconnect() {
    throw new TypeError('The method "disconnect" is not overrided.');
  }

  callAll() {
    throw new TypeError('The method "callAll" is not overrided.');
  }

  // eslint-disable-next-line no-unused-vars
  callIds(ids) {
    throw new TypeError('The method "callIds" is not overrided.');
  }

  closeAllRoomPeerConnections() {
    throw new TypeError('The method "closeAllRoomPeerConnections" is not overrided.');
  }

  // eslint-disable-next-line no-unused-vars
  callPeer(toId, offer) {
    throw new TypeError('The method "callPeer" is not overrided.');
  }

  // eslint-disable-next-line no-unused-vars
  makePeerCallAnswer(toId, answer) {
    throw new TypeError('The method "makePeerCallAnswer" is not overrided.');
  }

  // eslint-disable-next-line no-unused-vars
  rejectCall(toId) {
    throw new TypeError('The method "rejectCall" is not overrided.');
  }

  // eslint-disable-next-line no-unused-vars
  sendIceCandidate(toId, candidate) {
    throw new TypeError('The method "sendIceCandidate" is not overrided.');
  }
}

export default SignalingClient;
