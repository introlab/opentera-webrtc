import SignalingClient from './SignalingClient';

import io from 'socket.io-client';

const SignalingProtocolVersion = 1;

class SioSignalingClient extends SignalingClient {
  constructor(signalingServerConfiguration, logger) {
    super(signalingServerConfiguration, logger);

    this._socket = null;
  }

  get isConnected() {
    return this._socket !== null;
  }

  get sessionId() {
    if (this._socket !== null) {
      return this._socket.id;
    }
    else {
      return null;
    }
  }

  async connect() {
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
        this._logger('SignalingServer connect_error event: ', error);

        reject(error);
      });
      this._socket.on('connect_timeout', error => {
        this._logger('SignalingServer connect_timeout event: ', error);

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
        this._onSignalingConnectionError('Invalid password or invalid protocol version');
      }
    });
  }

  _connectEvents() {
    this._socket.on('disconnect', () => {
      this._logger('SignalingServer disconnect event');

      this._onSignalingConnectionClose();
    });

    this._socket.on('room-clients', clients => { this._onRoomClientsChange(clients); });

    this._socket.on('make-peer-call', async ids => await this._makePeerCall(ids));
    this._socket.on('peer-call-received', async data => await this._peerCallReceived(data));
    this._socket.on('peer-call-answer-received', async data => await this._peerCallAnswerReceived(data));
    this._socket.on('close-all-peer-connections-request-received', () => { this._closeAllPeerConnections(); });

    this._socket.on('ice-candidate-received', async data => await this._iceCandidateReceived(data));
  }

  disconnect() {
    this._disconnectEvents();
    this._socket.close();
    this._socket = null;
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

    this._socket.off('ice-candidate');
  }

  callAll() {
    this._socket.emit('call-all');
  }

  callIds(ids) {
    this._socket.emit('call-ids', ids);
  }

  closeAllRoomPeerConnections() {
    this._socket.emit('close-all-room-peer-connections');
  }

  callPeer(toId, offer) {
    let data = { toId: toId, offer: offer };
    this._socket.emit('call-peer', data);
  }

  makePeerCallAnswer(toId, answer) {
    let data = { toId: toId, answer: answer };
    this._socket.emit('make-peer-call-answer', data);
  }

  rejectCall(toId) {
    let data = { toId: toId };
    this._socket.emit('make-peer-call-answer', data);
  }

  sendIceCandidate(toId, candidate) {
    let data = { toId: toId, candidate: candidate };
    this._socket.emit('send-ice-candidate', data);
  }
}

export default SioSignalingClient;
