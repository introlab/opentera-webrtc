import adapter from 'webrtc-adapter'; // eslint-disable-line

import devices from './devices';
import iceServers from './iceServers';
import DataChannelClient from './DataChannelClient';
import StreamClient from './StreamClient';


let openteraWebrtcWebClient = {
  devices,
  iceServers,
  DataChannelClient,
  StreamClient
};

export default openteraWebrtcWebClient;
