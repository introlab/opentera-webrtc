describe('Wrong password DataChannelClient', () => {
  const SignallingServerConfiguration = {
    url: 'http://localhost:8080',
    name: '',
    room: 'chat',
    password: ''
  };
  const DataChannelConfiguration = {};
  const RtcConfiguration = {};

  it('connect should return an error', done => {
    let dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration,
      DataChannelConfiguration, RtcConfiguration);

    dataChannelClient.onSignallingConnectionOpen = () => {
      dataChannelClient.close();
      assert.fail();
      done();
    };
    dataChannelClient.onSignallingConnectionError = () => {
      expect(dataChannelClient.isConnected).to.eql(false);
      expect(dataChannelClient.isRtcConnected).to.eql(false);
      expect(dataChannelClient.id).to.eql(null);
      expect(dataChannelClient.connectedRoomClientIds).to.eql([]);
      expect(dataChannelClient.roomClients).to.eql([]);
      done();
    };

    dataChannelClient.connect();
  });
});