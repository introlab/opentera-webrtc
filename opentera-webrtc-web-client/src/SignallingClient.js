import io from 'socket.io-client';


class SignallingClient {
  constructor(signallingServerConfiguration, rtcPeerConnection, name, clientType) {
    if (!window.RTCSessionDescription) {
      throw new Error('RTCSessionDescription is not supported.');
    }

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._rtcPeerConnection = rtcPeerConnection;
    this._name = name;
    this._clientType = clientType;

    this._socket = null;
    this._isReady = false;

    this._onConnectionOpen = () => {};
    this._onConnectionClose = () => {};
    this._onReadyChanged = () => {};
  }
  
  async connect() {
    this._socket = io(this._signallingServerConfiguration.url);
    this._connectEvents();

    await new Promise((resolve, reject) => {
      this._socket.on('connect', () => resolve());
      this._socket.on('connect_error', error => reject(error));
      this._socket.on('connect_timeout', error_1 => reject(error_1));
    });
    await new Promise((resolve, reject) => {
      let data = { client_type: this._clientType, room: this._name };
      this._socket.emit('join-room', data, isJoined => {
        if (isJoined) {
          resolve();
          this._onConnectionOpen();
        }
        else {
          reject();
        }
      });
    });
  }  

  _connectEvents() {
    this._socket.on('disconnect', () => this._disconnect());

    this._socket.on('room-not-ready', () => this._roomNotReady());
    this._socket.on('room-ready', () => this._roomReady());

    this._socket.on('call-received', offer => this._callReceived(offer));
    this._socket.on('call-answer-received', answer => this._callAnswerReceived(answer));

    this._rtcPeerConnection.onicecandidate = event => this._socket.emit('ice-candidate', event.candidate);
    this._socket.on('ice-candidate', candidate => this._rtcPeerConnection.addIceCandidate(candidate));
  }

  _disconnect() {
    this._socket.close();
    this._socket = null;
    this._onConnectionClose();
  }

  _roomNotReady() {
    this._isReady = false;
    this._onReadyChanged(false);
  }

  _roomReady() {
    this._isReady = true;
    this._onReadyChanged(true);
  }

  async _callReceived(offer) {
    await this._rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(offer));
    
    let answer = await this._rtcPeerConnection.createAnswer();    
    await this._rtcPeerConnection.setLocalDescription(new window.RTCSessionDescription(answer));
    this._socket.emit('make-call-answer', answer);
  }

  _callAnswerReceived(answer) {
    this._rtcPeerConnection.setRemoteDescription(new window.RTCSessionDescription(answer));
  }

  async call() {
    let offer = await this._rtcPeerConnection.createOffer();
    await this._rtcPeerConnection.setLocalDescription(new RTCSessionDescription(offer));
    this._socket.emit('call', offer);
  }

  close() {
    this._socket.close();
  }

  get isReady() {
    return this._isReady;
  }

  set onConnectionOpen(onConnectionOpen) {
    this._onConnectionOpen = onConnectionOpen;
  }

  set onConnectionClose(onConnectionClose) {
    this._onConnectionClose = onConnectionClose;
  }

  set onReadyChanged(onReadyChanged) {
    this._onReadyChanged = onReadyChanged;
  }
}

export default SignallingClient;
