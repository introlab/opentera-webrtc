import SignalingClient from './SignalingClient';

const SignalingProtocolVersion = 1;

function _event_to_message(event, data) {
  if (data === undefined) {
    return JSON.stringify({event: event});
  } else {
    return JSON.stringify({event: event, data: data});
  }
}

class WebSocketSignalingClient extends SignalingClient {
  constructor(signalingServerConfiguration, logger) {
    super(signalingServerConfiguration, logger);
    this._ws = null;
    this._id = null;
  }

  get isConnected() {
    return this._id !== null;
  }

  get sessionId() {
    return this._id;
  }

  async connect() {
    this._ws = new WebSocket(this._signalingServerConfiguration.url);
    this._connectEvents();
  }

  _connectEvents() {
    this._ws.onopen = () => {
      this._logger('SignalingServer connect event');
      let data = {
        name: this._signalingServerConfiguration.name,
        data: this._signalingServerConfiguration.data,
        room: this._signalingServerConfiguration.room,
        password: this._signalingServerConfiguration.password,
        protocolVersion: SignalingProtocolVersion
      };
      this._ws.send(_event_to_message('join-room', data));
    };
    this._ws.onclose = event => {
      this._logger('SignalingServer disconnect event: ', event);
      this._onSignalingConnectionClose();
    };
    this._ws.onerror = event => {
      this._logger('SignalingServer error event: ', event);
      this._onSignalingConnectionError(event);
    };

    this._ws.onmessage = async event => {
      try {
        let data = JSON.parse(event.data);
        let eventName = data.event;
        data = data.data;

        switch (eventName)
        {
        case 'join-room-answer':
          this._handle_join_room_answer(data);
          break;
        case 'room-clients':
          this._onRoomClientsChange(data);
          break;
        case 'make-peer-call':
          await this._makePeerCall(data);
          break;
        case 'peer-call-received':
          await this._peerCallReceived(data);
          break;
        case 'peer-call-answer-received':
          await this._peerCallAnswerReceived(data);
          break;
        case 'close-all-peer-connections-request-received':
          this._closeAllPeerConnections();
          break;
        case 'ice-candidate-received':
          this._iceCandidateReceived(data);
          break;
        }
      } catch(error) {
        this._logger('Message error: ', error);
      }
    };
  }

  _handle_join_room_answer(id){
    if (id === '') {
      this._onSignalingConnectionError('Invalid password or invalid protocol version');
    } else {
      this._onSignalingConnectionOpen();
      this._id = id;
    }
  }

  disconnect() {
    this._disconnectEvents();
    this._id = null;
    this._ws.close();
    this._ws = null;
  }

  _disconnectEvents() {
    this._ws.onopen = () => {};
    this._ws.onclose = () => {};
    this._ws.onerror = () => {};
    this._ws.onmessage = () => {};
  }

  callAll() {
    this._ws.send(_event_to_message('call-all'));
  }

  callIds(ids) {
    this._ws.send(_event_to_message('call-ids', ids));
  }

  closeAllRoomPeerConnections() {
    this._ws.send(_event_to_message('close-all-room-peer-connections'));
  }

  callPeer(toId, offer) {
    let data = { toId: toId, offer: offer };
    this._ws.send(_event_to_message('call-peer', data));
  }

  makePeerCallAnswer(toId, answer) {
    let data = { toId: toId, answer: answer };
    this._ws.send(_event_to_message('make-peer-call-answer', data));
  }

  rejectCall(toId) {
    let data = { toId: toId };
    this._ws.send(_event_to_message('make-peer-call-answer', data));
  }

  sendIceCandidate(toId, candidate) {
    let data = { toId: toId, candidate: candidate };
    this._ws.send(_event_to_message('send-ice-candidate', data));
  }
}

export default WebSocketSignalingClient;
