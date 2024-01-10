describe('Wrong password DataChannelClient', () => {
  const SignalingServerConfiguration = {
    url: 'ws://localhost:8080/signaling',
    name: '',
    room: 'chat',
    password: ''
  };
  const DataChannelConfiguration = {};
  const RtcConfiguration = {};

  it('connect should return an error', done => {
    let dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignalingServerConfiguration,
      DataChannelConfiguration, RtcConfiguration);

    dataChannelClient.onSignalingConnectionOpen = () => {
      dataChannelClient.close();
      assert.fail();
      done();
    };
    dataChannelClient.onSignalingConnectionError = () => {
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
