const SignallingServerConfiguration = {
  url: 'http://localhost:8080',
  name: '',
  room: 'chat',
  password: ''
};
const DataChannelConfiguration = {};
const RtcConfiguration = {};

let dataChannelClient;

describe('Disconnected DataChannelClient', () => {
  beforeEach(() => dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration,
    DataChannelConfiguration, RtcConfiguration));
    
  it('isConnected should return false', () => {
    expect(dataChannelClient.isConnected).to.eql(false);
  });
  it('isRtcConnected should return false', () => {
    expect(dataChannelClient.isRtcConnected).to.eql(false);
  });
  it('id should return null', () => {
    expect(dataChannelClient.id).to.eql(null);
  });
  it('connectedRoomClientIds should return []', () => {
    expect(dataChannelClient.connectedRoomClientIds).to.eql([]);
  });
  it('roomClients should return []', () => {
    expect(dataChannelClient.roomClients).to.eql([]);
  });
});
