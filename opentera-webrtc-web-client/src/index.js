import adapter from 'webrtc-adapter'; // eslint-disable-line

import devices from './devices';
import iceServers from './iceServers';
import DataChannelClient from './DataChannelClient';


let openteraWebrtcWebClient = {
  devices,
  iceServers,
  DataChannelClient
};

export default openteraWebrtcWebClient;
