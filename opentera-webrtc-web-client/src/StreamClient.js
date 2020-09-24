import devices from './devices';
import SignallingClient from './SignallingClient';


function getUserMediaConstraintsFromStreamConfiguration(configuration) {
  let audio = false;
  let video = false;
  
  if (configuration.audioDeviceId) {
    audio = { deviceId: configuration.audioDeviceId };
  }
  if (configuration.audioConstraints) {
    audio = { ...audio, ...configuration.audioConstraints };
  }

  if (configuration.videoDeviceId) {
    video = { deviceId: configuration.videoDeviceId };
  }
  if (configuration.videoConstraints) {
    video = { ...video, ...configuration.videoConstraints };
  }

  return { audio, video };
}


class StreamClient
{
  constructor(signallingServerConfiguration, streamConfiguration, rtcConfiguration, clientType)
  {
    if (!window.RTCPeerConnection) {
      throw new Error('RTCPeerConnection is not supported.');
    }

    if (!signallingServerConfiguration) {
      throw new Error('signallingServerConfiguration is required');
    }
    if (!streamConfiguration) {
      throw new Error('streamConfiguration is required');
    }
    if (!rtcConfiguration) {
      rtcConfiguration = {};
    }

    this._signallingServerConfiguration = signallingServerConfiguration;
    this._streamConfiguration = streamConfiguration;
    this._rtcConfiguration = rtcConfiguration;

    this._rtcPeerConnection = null;

    this._localStream = null;
    this._remoteStream = null;

    this._onSignallingConnectionOpen = () => {};
    this._onSignallingConnectionClose = () => {};
    this._onPeerReadyChanged = () => {};

    this._onLocalStream = () => {};
    this._onRemoteStream = () => {};

    this._signallingClient = new SignallingClient(signallingServerConfiguration, this._rtcPeerConnection,
      streamConfiguration.name, clientType);
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

    if (this._streamConfiguration.stream) {
      this._localStream = this._streamConfiguration.stream;
    }
    else if (this._streamConfiguration.audioDeviceId || this._streamConfiguration.videoDeviceId) {
      let constraints = getUserMediaConstraintsFromStreamConfiguration(this._streamConfiguration);
      this._localStream = await devices.getStream(constraints);
      
      this._localStream.getTracks().forEach(track => this._rtcPeerConnection.addTrack(track, this._localStream));
      this._onLocalStream(this._localStream);
    }

    this._connectRtcPeerConnectionEvents();

    await this._signallingClient.connect();
  }

  _connectRtcPeerConnectionEvents() {
    this._rtcPeerConnection.ontrack = ({ streams: [stream] }) => {
      this._remoteStream = stream;
      this._onRemoteStream(stream);
    };
    this._rtcPeerConnection.onconnectionstatechange = () => {
      switch (this._rtcPeerConnection.connectionState) {
      case 'disconnected':
      case 'failed':
      case 'closed':
        this.close();
        break;
      }
    };
    this._rtcPeerConnection.oniceconnectionstatechange = () => {
      switch (this._rtcPeerConnection.iceConnectionState) {
      case 'disconnected':
      case 'failed':
      case 'closed':
        this.close();
        break;
      }
    };
  }

  _disconnectRtcPeerConnectionEvents(rtcPeerConnection) {
    rtcPeerConnection.ontrack = () => {};
    rtcPeerConnection.onconnectionstatechange = () => {};
    rtcPeerConnection.oniceconnectionstatechange = () => {};
  }

  async call() {
    this._signallingClient.call();
  }

  close () {
    if (this._rtcPeerConnection !== null) {
      this._localStream.getTracks().forEach(track => track.stop());
      this._remoteStream.getTracks().forEach(track => track.stop());

      this._localStream = null;
      this._remoteStream = null;

      let signallingClient = this._signallingClient;
      this._signallingClient = null;
      signallingClient.close();
      
      let rtcPeerConnection = this._rtcPeerConnection;
      this._rtcPeerConnection = null;
      rtcPeerConnection.close();
      this._disconnectRtcPeerConnectionEvents(rtcPeerConnection);
    }
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

  get localStream() {
    return this._localStream;
  }

  get remoteStream() {
    return this._remoteStream;
  }

  set onLocalStream(onLocalStream) {
    this._onLocalStream = onLocalStream;
  }

  set onRemoteStream(onRemoteStream) {
    this._onRemoteStream = onRemoteStream;
  }
}

export default StreamClient;
