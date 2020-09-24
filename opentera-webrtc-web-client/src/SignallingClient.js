import io from 'socket.io-client';


class SignallingClient {
  constructor(signallingServerConfiguration, name, room, getRtcPeerConnection, getAllRtcPeerConnection) {
    if (!window.RTCSessionDescription) {
      throw new Error('RTCSessionDescription is not supported.');
    }

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._getRtcPeerConnection = getRtcPeerConnection;
    this._getAllRtcPeerConnection = getAllRtcPeerConnection;
    this._name = name;
    this._room = room;

    this._socket = null;

    this._onConnectionOpen = () => {};
    this._onConnectionClose = () => {};
    this._onRoomClientsChanged = () => {};
  }
  
  async connect() {
    this._socket = io(this._signallingServerConfiguration.url);
    this._connectEvents();

    await new Promise((resolve, reject) => {
      this._socket.on('connect', () => {
        resolve();
        this._onConnectionOpen();
      });
      this._socket.on('connect_error', error => reject(error));
      this._socket.on('connect_timeout', error => reject(error));
    });
    
    let data = { name: this._name, room: this._room };
    this._socket.emit('join-room', data);
  }  

  _connectEvents() {
    this._socket.on('disconnect', () => this._disconnect());

    this._socket.on('room-clients', clients => this._onRoomClientsChanged(clients));

    this._socket.on('make-calls', async ids => await this._makeCalls(ids));
    this._socket.on('call-received', async data => await this._callReceived(data));
    this._socket.on('call-answer-received', async data => await this._callAnswerReceived(data));

    this._socket.on('ice-candidate-received', async data => await this._addIceCandidate(data));
  }

  _disconnectEvents() {
    this._socket.off('connect');
    this._socket.off('connect_error');
    this._socket.off('connect_timeout');
    this._socket.off('disconnect');

    this._socket.off('room-clients');

    this._socket.off('make-calls');
    this._socket.off('call-received');
    this._socket.off('call-answer-received');

    this._getAllRtcPeerConnection().forEach(c => c.onicecandidate = () => {});
    this._socket.off('ice-candidate');
  }

  _disconnect() {
    this._disconnectEvents();
    this._socket.close();
    this._socket = null;
    this._onConnectionClose();
  }

  async _callReceived(data) {
    let rtcPeerConnection = this._getRtcPeerConnection(data.fromId, false);
    this._connectOnIceCandidateEvent(data.fromId, rtcPeerConnection);

    await rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.offer));
    
    let answer = await rtcPeerConnection.createAnswer();    
    await rtcPeerConnection.setLocalDescription(new window.RTCSessionDescription(answer));

    data = { toId: data.fromId, answer: answer };
    this._socket.emit('make-call-answer', data);
  }

  async _callAnswerReceived(data) {
    let _rtcPeerConnection =  this._getRtcPeerConnection(data.fromId, false);
    await _rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(data.answer));
  }

  async _addIceCandidate(data) {
    if (data && data.candidate) {
      this._getRtcPeerConnection(data.fromId).addIceCandidate(data.candidate);
    }
  }

  async _makeCalls(ids) {
    ids = ids.filter(id => id != this._socket.id);
    ids.forEach(async id => {
      let rtcPeerConnection = this._getRtcPeerConnection(id, true);
      this._connectOnIceCandidateEvent(id, rtcPeerConnection);

      let offer = await rtcPeerConnection.createOffer();
      await rtcPeerConnection.setLocalDescription(new RTCSessionDescription(offer));

      let data = { toId: id, offer: offer };
      this._socket.emit('call', data);
    });
  }

  _connectOnIceCandidateEvent(id, rtcPeerConnection) {
    rtcPeerConnection.onicecandidate = event => {
      let data = { toId: id, candidate: event.candidate };
      this._socket.emit('send-ice-candidate', data);
    };
  }

  close() {
    this._socket.close();
  }

  callAll() {
    this._socket.emit('call-all');
  }

  hangUp() {
    this._getAllRtcPeerConnection().forEach(c => c.onicecandidate = () => {});
  }

  set onConnectionOpen(onConnectionOpen) {
    this._onConnectionOpen = onConnectionOpen;
  }

  set onConnectionClose(onConnectionClose) {
    this._onConnectionClose = onConnectionClose;
  }

  set onRoomClientsChanged(onRoomClientsChanged) {
    this._onRoomClientsChanged = onRoomClientsChanged;
  }
}

export default SignallingClient;
