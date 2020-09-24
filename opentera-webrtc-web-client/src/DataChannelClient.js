import SignallingClient from './SignallingClient';


class DataChannelClient {
  constructor(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration, name, room) {
    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }

    if (!signallingServerConfiguration) {
      throw new Error('signallingServerConfiguration is required');
    }
    if (!dataChannelConfiguration) {
      throw new Error('dataChannelConfiguration is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._dataChannelConfiguration = dataChannelConfiguration;
    this._rtcConfiguration = rtcConfiguration;
    this._name = name;
    this._room = room;

    this._rtcPeerConnections = {};
    this._dataChannels = {};
    this._clientNames = {};

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};

    this._onRoomClientsChanged = () => {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (id, event) => {
      console.error('Data channel error: ' + id + ', ' + event);
    };
  }

  async connect() {
    let getRtcPeerConnection = (id, isCaller) => this._getRtcPeerConnection(id, isCaller);
    let getAllRtcPeerConnection = () => this._getAllRtcPeerConnection();
    this._signallingClient = new SignallingClient(this._signallingServerConfiguration, this._name, this._room,
      getRtcPeerConnection, getAllRtcPeerConnection);

    this._signallingClient.onConnectionOpen = this._onSignallingConnectionOpen;
    this._signallingClient.onConnectionClose = () => {
      this.close();
      this._onSignallingConnectionClose();
    };
    this._signallingClient.onRoomClientsChanged = clients => {
      this._onRoomClientsChanged(clients);
      this._updateClientNames(clients);
    };

    await this._signallingClient.connect();
  }

  _getRtcPeerConnection(id, isCaller) {
    if (id in this._rtcPeerConnections) {
      return this._rtcPeerConnections[id];
    }
    else {
      let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

      if (isCaller) {
        let dataChannel = rtcPeerConnection.createDataChannel(this._room, this._dataChannelConfiguration);
        this._dataChannels[id] = dataChannel;
        this._connectDataChannelEvents(id, dataChannel);
      }
      else {
        this._connectDataChannelsRtcPeerConnectionEvents(id, rtcPeerConnection);
      }

      this._rtcPeerConnections[id] = rtcPeerConnection;
      return rtcPeerConnection;
    }
  } 
  
  _getAllRtcPeerConnection() {
    let rtcPeerConnections = [];
    for (let id in this._rtcPeerConnections) {
      rtcPeerConnections.push(this._rtcPeerConnections[id]);
    }
    return rtcPeerConnections;
  }

  _connectDataChannelsRtcPeerConnectionEvents(id, rtcPeerConnection) {
    rtcPeerConnection.ondatachannel = ({ channel: dataChannel }) => {
      this._dataChannels[id] = dataChannel;
      this._connectDataChannelEvents(id, dataChannel);
    };
  }

  _disconnectDataChannelsRtcPeerConnectionEvents(rtcPeerConnection) {
    rtcPeerConnection.ondatachannel = () => {};
  }

  _connectDataChannelEvents(id, dataChannel) {
    dataChannel.onmessage = event => {
      this._onDataChannelMessage(id, this._clientNames[id], event.data);
    };
    dataChannel.onopen = event => {
      this._onDataChannelOpen(id, this._clientNames[id], event);
    };
    dataChannel.onclose = event => {
      this._removeConnection(id);
      this._onDataChannelClose(id, this._clientNames[id], event);
    };
    dataChannel.onerror = event => {
      this._removeConnection(id);
      this._onDataChannelError(id, this._clientNames[id], event);
      this._onDataChannelClose(id, this._clientNames[id], event);
    };
  }

  _disconnectDataChannelEvents(dataChannel) {
    dataChannel.onmessage = () => {};
    dataChannel.onopen = () => {};
    dataChannel.onclose = () => {};
    dataChannel.onerror = () => {};
  }

  _updateClientNames(clients) {
    this._clientNames = {};
    clients.forEach(client => {
      this._clientNames[client.id] = client.name;
    });
  }

  _removeConnection(id) {
    this._dataChannels[id].close();
    this._disconnectDataChannelEvents(this._dataChannels[id]);
    delete this._dataChannels[id];

    this._rtcPeerConnections[id].close();
    this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
    delete this._rtcPeerConnections[id];
  }

  async callAll() {
    this._signallingClient.callAll();
  }

  hangUp() {
    this._signallingClient.hangUp();

    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
    }
    this._dataChannels = {};

    for (let id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
    }
    this._rtcPeerConnections = {};
  }

  close () {
    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
    }
    this._dataChannels = {};

    if (this._signallingClient !== null) {
      let signallingClient = this._signallingClient;
      this._signallingClient = null;
      signallingClient.close();
    }
    
    for (let id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
    }
    this._rtcPeerConnections = {};
  }

  sendTo(data, id) {
    this._dataChannels[id].send(data);
  }

  sendToAll(data) {
    for (let id in this._dataChannels) {
      this._dataChannels[id].send(data);
    }
  }

  get isConnected() {
    return Object.keys(this._rtcPeerConnections).length > 0;
  }

  set onSignallingConnectionOpen(onSignallingConnectionOpen) {
    this._onSignallingConnectionOpen = onSignallingConnectionOpen;
  }

  set onSignallingConnectionClose(onSignallingConnectionClose) {
    this._onSignallingConnectionClose = onSignallingConnectionClose;
  }

  set onRoomClientsChanged(onRoomClientsChanged) {
    this._onRoomClientsChanged = onRoomClientsChanged;
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

export default DataChannelClient;
