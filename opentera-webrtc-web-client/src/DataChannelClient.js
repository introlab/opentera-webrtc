import WebrtcClient from './WebrtcClient';

/**
 * @brief Represents a client for data channel communication.
 */
class DataChannelClient extends WebrtcClient {
  /**
   * @brief Creates a data channel client with the specified configurations.
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
   * @param {Object} dataChannelConfiguration The data channel configuration
   *  See https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/createDataChannel options parameter
   *
   * @param {Object} rtcConfiguration The RTC configuration
   *  See https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/RTCPeerConnection configuration parameter
   *
   * @param {CallableFunction} logger An optional logger callback
   */
  constructor(signalingServerConfiguration, dataChannelConfiguration, rtcConfiguration, logger) {
    super(signalingServerConfiguration, logger);

    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }

    if (!dataChannelConfiguration) {
      throw new Error('dataChannelConfiguration is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._dataChannelConfiguration = dataChannelConfiguration;
    this._rtcConfiguration = rtcConfiguration;
    this._rtcConfiguration['sdpSemantics'] = 'unified-plan';

    this._dataChannels = {};

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
      let dataChannel = rtcPeerConnection.createDataChannel(this._signalingClient.room,
        this._dataChannelConfiguration);
      this._dataChannels[id] = dataChannel;
      this._connectDataChannelEvents(id, dataChannel);
    }

    return rtcPeerConnection;
  }

  _connectRtcPeerConnectionEvents(id, rtcPeerConnection) {
    super._connectRtcPeerConnectionEvents(id, rtcPeerConnection);

    if (!(id in this._dataChannels)) {
      rtcPeerConnection.ondatachannel = ({ channel: dataChannel }) => {
        this._dataChannels[id] = dataChannel;
        this._connectDataChannelEvents(id, dataChannel);
      };
    }
  }

  _disconnectRtcPeerConnectionEvents(rtcPeerConnection) {
    super._disconnectRtcPeerConnectionEvents(rtcPeerConnection);

    rtcPeerConnection.ondatachannel = () => {};
  }

  _connectDataChannelEvents(id, dataChannel) {
    dataChannel.onmessage = event => {
      this._onDataChannelMessage(id, this.getClientName(id), this.getClientData(id), event.data);
    };
    dataChannel.onopen = () => {
      this._logger('onDataChannelOpen, id=', id);

      this._onDataChannelOpen(id, this.getClientName(id), this.getClientData(id));
      this.updateRoomClients();
    };
    dataChannel.onclose = () => {
      this._logger('onDataChannelClose, id=', id);

      this._removeConnection(id);
      this.updateRoomClients();
    };
    dataChannel.onerror = event => {
      this._logger('onDataChannelError, id=', id);

      this._removeConnection(id);
      this._onDataChannelError(id, this.getClientName(id), this.getClientData(id), event);
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
    super._removeConnection(id);

    if (id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      delete this._dataChannels[id];
      this._onDataChannelClose(id, this.getClientName(id), this.getClientData(id));
    }
  }

  _closeAllDataChannels() {
    for (let id in this._dataChannels) {
      this._dataChannels[id].close();
      this._disconnectDataChannelEvents(this._dataChannels[id]);
      delete this._dataChannels[id];
      this._onDataChannelClose(id, this.getClientName(id), this.getClientData(id));
    }
  }

  /**
   * @brief Hangs up all clients.
   */
  hangUpAll() {
    super.hangUpAll();
    this._closeAllDataChannels();
  }

  /**
   * @brief Closes all client connections
   */
  close() {
    this._closeAllDataChannels();
    super.close();
  }

  /**
   * @brief Sends binary data to the specified clients.
   *
   * @param {String | Blob | ArrayBuffer} data The data
   * @param {Array<String>} ids The client ids
   */
  sendTo(data, ids) {
    ids.forEach(id => this._dataChannels[id].send(data));
  }

  /**
   * @brief Sends binary data to all clients.
   * @param {String | Blob | ArrayBuffer} data
   */
  sendToAll(data) {
    for (let id in this._dataChannels) {
      this._dataChannels[id].send(data);
    }
  }

  /**
   * @brief Sets the callback that is called when data are received.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   *  - data: The message data
   * @endparblock
   *
   * @param {CallableFunction} onDataChannelMessage The callback
   */
  set onDataChannelMessage(onDataChannelMessage) {
    this._onDataChannelMessage = onDataChannelMessage;
  }

  /**
   * @brief Sets the callback that is called when a data channel opens.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param {CallableFunction} onDataChannelOpen The callback
   */
  set onDataChannelOpen(onDataChannelOpen) {
    this._onDataChannelOpen = onDataChannelOpen;
  }

  /**
   * @brief Sets the callback that is called when a data channel closes.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - clientName: The client name
   *  - clientData: The client data
   * @endparblock
   *
   * @param {CallableFunction} onDataChannelClose
   */
  set onDataChannelClose(onDataChannelClose) {
    this._onDataChannelClose = onDataChannelClose;
  }

  /**
   * @brief Sets the callback that is called when a data channel error occurs.
   *
   * @parblock
   * Callback parameters:
   *  - clientId: The client id
   *  - event: The error event
   * @endparblock
   *
   * @param {CallableFunction} onDataChannelError
   */
  set onDataChannelError(onDataChannelError) {
    this._onDataChannelError = onDataChannelError;
  }
}

export default DataChannelClient;
