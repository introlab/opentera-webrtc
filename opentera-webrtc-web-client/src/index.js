import adapter from 'webrtc-adapter'; // eslint-disable-line

import devices from './devices';
import iceServers from './iceServers';
import DataChannelClient from './DataChannelClient';
import StreamClient from './StreamClient';
import StreamDataChannelClient from './StreamDataChannelClient';


let openteraWebrtcWebClient = {
  devices,
  iceServers,
  DataChannelClient,
  StreamClient,
  StreamDataChannelClient
};

export default openteraWebrtcWebClient;
