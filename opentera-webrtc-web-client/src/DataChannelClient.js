import SignallingClient from './SignallingClient';


class DataChannelClient {
  constructor(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration, clientType) {
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
    this._clientType = clientType;

    this._rtcPeerConnection = null;
    this._dataChannel = null;

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onPeerReadyChanged = () => {};

    this._onDataChannelMessage = () => {};
    this._onDataChannelOpen = () => {};
    this._onDataChannelClose = () => {};
    this._onDataChannelError = (event) => {
      console.error('Data channel error: ' + event);
    };
  }

  async connect() {
    this._rtcPeerConnection = new window.RTCPeerConnection(this._rtcConfiguration);
    this._signallingClient = new SignallingClient(this._signallingServerConfiguration, this._rtcPeerConnection,
      this._dataChannelConfiguration.name, this._clientType);

    this._signallingClient.onConnectionOpen = this._onSignallingConnectionOpen;
    this._signallingClient.onConnectionClose = () => {
      this._signallingClient = null;
      this._onSignallingConnectionClose();
    };
    this._signallingClient.onReadyChanged = this._onPeerReadyChanged;

    if (!this._dataChannelConfiguration.isRemote) {
      let channel = this._rtcPeerConnection.createDataChannel(this._dataChannelConfiguration.name,
        this._dataChannelConfiguration.options);
      this._dataChannel = channel;
      this._connectDataChannelEvents();
    }
    else {
      this._connectDataChannelsRtcPeerConnectionEvents();
    }

    await this._signallingClient.connect();
  }

  _connectDataChannelsRtcPeerConnectionEvents() {
    this._rtcPeerConnection.ondatachannel = ({ channel: channel }) => {
      this._dataChannel = channel;
      this._connectDataChannelEvents(channel);
    };
  }

  _connectDataChannelEvents() {
    this._dataChannel.onmessage = event => {
      this._onDataChannelMessage(event.data);
    };
    this._dataChannel.onopen = event => {
      this._onDataChannelOpen(event);
    };
    this._dataChannel.onclose = event => {
      this._dataChannel = null;

      if (this._rtcPeerConnection !== null) {
        this.close();
        this._onDataChannelClose(event);
      }
    };
    this._dataChannel.onerror = event => {
      this._onDataChannelError(event);
    };
  }

  async call() {
    this._signallingClient.call();
  }

  close () {
    if (this._dataChannel !== null) {
      this._dataChannel.close();
      this._dataChannel = null;
    }

    if (this._rtcPeerConnection !== null) {
      let signallingClient = this._signallingClient;
      this._signallingClient = null;
      signallingClient.close();
      
      let rtcPeerConnection = this._rtcPeerConnection;
      this._rtcPeerConnection = null;
      rtcPeerConnection.close();
    }
  }

  send(data) {
    if (this._dataChannel !== null) {
      this._dataChannel.send(data);
    }
    else {
      console.error('Not connected data channel');
    }
  }

  get dataChannel() {
    return this._dataChannel;
  }

  get isReady() {
    if (this._signallingClient !== null) {
      return this._signallingClient.isReady;
    }
    return false;
  }

  set onSignallingConnectionOpen(onSignallingConnectionOpen) {
    this._onSignallingConnectionOpen = onSignallingConnectionOpen;
  }

  set onSignallingConnectionClose(onSignallingConnectionClose) {
    this._onSignallingConnectionClose = onSignallingConnectionClose;
  }

  set onPeerReadyChanged(onPeerReadyChanged) {
    this._onPeerReadyChanged = onPeerReadyChanged;
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
