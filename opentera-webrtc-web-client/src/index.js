import adapter from 'webrtc-adapter'; // eslint-disable-line

import devices from './devices';
import DataChannelClient from './DataChannelClient';


let opentera_webrtc_web_client = {
  devices,
  DataChannelClient
};

export default opentera_webrtc_web_client;
