import SignallingClient from './SignallingClient';


class DataChannelClient extends SignallingClient {
  constructor(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration) {
    super(signallingServerConfiguration);

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

    this._dataChannels = {};

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onSignallingConnectionError = () => {};

    this._onRoomClientsChanged = () => {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (id, event) => {
      console.error('Data channel error: ' + id + ', ' + event);
    };
  }

  _createRtcPeerConnection(id, isCaller) {
    let rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);

    if (isCaller) {
      let dataChannel = rtcPeerConnection.createDataChannel(this._signallingServerConfiguration.room,
        this._dataChannelConfiguration);
      this._dataChannels[id] = dataChannel;
      this._connectDataChannelEvents(id, dataChannel);
    }
    else {
      this._connectDataChannelsRtcPeerConnectionEvents(id, rtcPeerConnection);
    }

    return rtcPeerConnection;
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
      this._onDataChannelMessage(id, this.getClientName(id), event.data);
    };
    dataChannel.onopen = event => {
      this._onDataChannelOpen(id, this.getClientName(id), event);
      this.updateRoomClients();
    };
    dataChannel.onclose = event => {
      this._removeConnection(id);
      this._onDataChannelClose(id, this.getClientName(id), event);
      this.updateRoomClients();
    };
    dataChannel.onerror = event => {
      this._removeConnection(id);
      this._onDataChannelError(id, this.getClientName(id), event);
      this._onDataChannelClose(id, this.getClientName(id), event);
      this.updateRoomClients();
    };
  }

  _disconnectDataChannelEvents(dataChannel) {
    dataChannel.onmessage = () => {};
    dataChannel.onopen = () => {};
    dataChannel.onclose = () => {};
    dataChannel.onerror = () => {};
  }

  _removeConnection(id) {
    if (id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      delete this._dataChannels[id];
    }

    if (id in this._rtcPeerConnections) {
      this._rtcPeerConnections[id].close();
      this._disconnectDataChannelsRtcPeerConnectionEvents(this._rtcPeerConnections[id]);
      delete this._rtcPeerConnections[id];
    }
  }

  _closeAllDataChannels() {
    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      this._onDataChannelClose(id, this.getClientName(id), {});
    }
    this._dataChannels = {};
  }

  hangUpAll() {
    super.hangUpAll();
    this._closeAllDataChannels();
  }

  close () {
    this._closeAllDataChannels();
    super.close();
  }

  sendTo(data, ids) {
    ids.forEach(id => this._dataChannels[id].send(data));
  }

  sendToAll(data) {
    for (let id in this._dataChannels) {
      this._dataChannels[id].send(data);
    }
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
