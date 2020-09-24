import DataChannelClient from './DataChannelClient';
import StreamClient from './StreamClient';


class Clients
{
  constructor(signallingServerConfiguration, streamConfigurations, dataChannelConfigurations, rtcConfiguration, clientType) {
    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }

    if (!signallingServerConfiguration) {
      throw new Error('signallingServerConfiguration is required');
    }
    if (!streamConfigurations) {
      throw new Error('streamConfigurations is required');
    }
    if (!dataChannelConfigurations) {
      throw new Error('dataChannelConfigurations is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._streamClients = streamConfigurations
      .map(streamConfiguration => new StreamClient(signallingServerConfiguration, streamConfiguration, rtcConfiguration, clientType));
    this._dataChannelClients = dataChannelConfigurations
      .map(dataChannelConfiguration => new DataChannelClient(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration, clientType));

    this._connectStreamClientEvents();
    this._connectDataChannelClientEvents();

    this._onStreamSignallingConnectionOpen = () => {};
    this._onStreamSignallingConnectionClose = () => {};
    this._onStreamPeerReadyChanged = () => {};

    this._onDataChannelSignallingConnectionOpen = () => {};
    this._onDataChannelSignallingConnectionClose = () => {};
    this._onDataChannelPeerReadyChanged = () => {};

    this._onLocalStream = () => {};
    this._onRemoteStream = () => {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (name, event) => {
      console.error('Data channel error (' + name + '): ' + event);
    };
  }

  _connectStreamClientEvents() {
    for (let name in this._streamClients) {
      this._streamClients[name].onSignallingConnectionOpen = event => this._onStreamSignallingConnectionOpen(name, event);
      this._streamClients[name].onSignallingConnectionClose = event => this._onStreamSignallingConnectionClose(name, event);
      this._streamClients[name].onPeerReadyChanged = event => this._onStreamPeerReadyChanged(name, event);

      this._streamClients[name].onLocalStream = event => this._onLocalStream(name, event);
      this._streamClients[name].onRemoteStream = event => this._onRemoteStream(name, event);
    }
  }

  _connectDataChannelClientEvents() {
    for (let name in this._dataChannelClients) {
      this._dataChannelClients[name].onSignallingConnectionOpen = event => this._onDataChannelSignallingConnectionOpen(name, event);
      this._dataChannelClients[name].onSignallingConnectionClose = event => this._onDataChannelSignallingConnectionClose(name, event);
      this._dataChannelClients[name].onPeerReadyChanged = event => this._onDataChannelPeerReadyChanged(name, event);

      this._dataChannelClients[name].onDataChannelMessage = event => this._onDataChannelMessage(name, event);
      this._dataChannelClients[name].onDataChannelOpen = event => this._onDataChannelOpen(name, event);
      this._dataChannelClients[name].onDataChannelClose = event => this._onDataChannelClose(name, event);
      this._dataChannelClients[name].onDataChannelError = event => this._onDataChannelError(name, event);
    }
  }

  async connect() {
    let streamClientPromises = this._streamClients.map(client => client.connect());
    let dataChannelClientPromises = this._dataChannelClients.map(client => client.connect());
    
    await Promise.all(streamClientPromises.concat(dataChannelClientPromises));
  }

  async call() {
    let streamClientPromises = this._streamClients.map(client => client.call());
    let dataChannelClientPromises = this._dataChannelClients.map(client => client.call());
    
    await Promise.all(streamClientPromises.concat(dataChannelClientPromises));
  }

  close() {
    this._streamClients.forEach(client => client.close());
    this._dataChannelClients.forEach(client => client.close());
  }

  getLocalStreams(name) {
    return this._streamClients[name].localStream;
  }

  getRemoteStreams(name) {
    return this._streamClients[name].remoteStream;
  }

  getDataChannels(name) {
    return this._dataChannelClients[name].dataChannel;
  }

  sendData(name, data) {
    this._dataChannelClients[name].send(data);
  }

  set onLocalStream(onLocalStream) {
    this._onLocalStream = onLocalStream;
  }

  set onStreamSignallingConnectionOpen(onStreamSignallingConnectionOpen) {
    this._onStreamSignallingConnectionOpen = onStreamSignallingConnectionOpen;
  }

  set onStreamSignallingConnectionClose(onStreamSignallingConnectionClose) {
    this._onStreamSignallingConnectionClose = onStreamSignallingConnectionClose;
  }

  set onStreamPeerReadyChanged(onStreamPeerReadyChanged) {
    this._onStreamPeerReadyChanged = onStreamPeerReadyChanged;
  }

  set onDataChannelSignallingConnectionOpen(onDataChannelSignallingConnectionOpen) {
    this._onDataChannelSignallingConnectionOpen = onDataChannelSignallingConnectionOpen;
  }

  set onDataChannelSignallingConnectionClose(onDataChannelSignallingConnectionClose) {
    this._onDataChannelSignallingConnectionClose = onDataChannelSignallingConnectionClose;
  }

  set onDataChannelPeerReadyChanged(onDataChannelPeerReadyChanged) {
    this._onDataChannelPeerReadyChanged = onDataChannelPeerReadyChanged;
  }

  set onRemoteStream(onRemoteStream) {
    this._onRemoteStream = onRemoteStream;
  }

  set onDataChannelMessage(onDataChannelMessage) {
    this._onDataChannelMessage = onDataChannelMessage;
  }

  set onDataChannelOpen(onDataChannelOpen) {
    this._onDataChannelOpen = onDataChannelOpen;
  }

  set onDataChannelClose(onDataChannelClose) {
    this._onDataChannelClose = onDataChannelClose;
  }

  set onDataChannelError(onDataChannelError) {
    this._onDataChannelError = onDataChannelError;
  }
}

export default Clients;
