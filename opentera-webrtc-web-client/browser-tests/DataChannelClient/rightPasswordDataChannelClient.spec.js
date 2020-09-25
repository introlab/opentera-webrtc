describe('Right password DataChannelClient', done => {
  let dataChannelClient1;
  let dataChannelClient2;
  let dataChannelClient3;

  const BEFORE_AFTER_TIMEOUT_MS = 100;
  const SignallingServerConfiguration1 = {
    url: 'http://localhost:8080',
    name: 'c1',
    room: 'chat',
    password: 'abc'
  };
  const SignallingServerConfiguration2 = {
    url: 'http://localhost:8080',
    name: 'c2',
    room: 'chat',
    password: 'abc'
  };
  const SignallingServerConfiguration3 = {
    url: 'http://localhost:8080',
    name: 'c3',
    room: 'chat',
    password: 'abc'
  };
  const DataChannelConfiguration = {};
  const RtcConfiguration = {};

  beforeEach(done => {
    let counter = 0;
    dataChannelClient1 = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration1,
      DataChannelConfiguration, RtcConfiguration);
    dataChannelClient2 = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration2,
      DataChannelConfiguration, RtcConfiguration);
    dataChannelClient3 = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration3,
      DataChannelConfiguration, RtcConfiguration);

    let onSignallingConnectionOpen = () => {
      counter++;
      if (counter >= 3) {
        setTimeout(done, BEFORE_AFTER_TIMEOUT_MS);
      }
    };
    let onSignallingConnectionError = () => {
      expect.fail();
      done();
    };
    dataChannelClient1.onSignallingConnectionOpen = onSignallingConnectionOpen;
    dataChannelClient1.onSignallingConnectionError = onSignallingConnectionError;
    dataChannelClient2.onSignallingConnectionOpen = onSignallingConnectionOpen;
    dataChannelClient2.onSignallingConnectionError = onSignallingConnectionError;
    dataChannelClient3.onSignallingConnectionOpen = onSignallingConnectionOpen;
    dataChannelClient3.onSignallingConnectionError = onSignallingConnectionError;

    dataChannelClient1.connect();
    dataChannelClient2.connect();
    dataChannelClient3.connect();
  });

  afterEach(done => {
    dataChannelClient1.close();
    dataChannelClient2.close();
    dataChannelClient3.close();
    setTimeout(done, BEFORE_AFTER_TIMEOUT_MS);
  });

  it('isConnected should return true', () => {
    expect(dataChannelClient1.isConnected).to.eql(true);
    expect(dataChannelClient2.isConnected).to.eql(true);
    expect(dataChannelClient3.isConnected).to.eql(true);
  });
  it('isRtcConnected should return false', () => {
    expect(dataChannelClient1.isRtcConnected).to.eql(false);
    expect(dataChannelClient2.isRtcConnected).to.eql(false);
    expect(dataChannelClient3.isRtcConnected).to.eql(false);
  });
  it('id should not return null', () => {
    expect(dataChannelClient1.id).to.not.eql(null);
    expect(dataChannelClient2.id).to.not.eql(null);
    expect(dataChannelClient3.id).to.not.eql(null);
  });
  it('connectedRoomClientIds should return []', () => {
    expect(dataChannelClient1.connectedRoomClientIds).to.eql([]);
    expect(dataChannelClient2.connectedRoomClientIds).to.eql([]);
    expect(dataChannelClient3.connectedRoomClientIds).to.eql([]);
  });
  it('roomClients should return all clients', () => {
    expect(dataChannelClient1.roomClients.length).to.eql(3);
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', isConnected: true});
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', isConnected: false});
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', isConnected: false});

    expect(dataChannelClient2.roomClients.length).to.eql(3);
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', isConnected: false});
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', isConnected: true});
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', isConnected: false});

    expect(dataChannelClient3.roomClients.length).to.eql(3);
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', isConnected: false});
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', isConnected: false});
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', isConnected: true});
  });

  it('callAll should call all clients', done => {
    let clientCounter = 0;
    let dataChannelCounter1 = 0;
    let dataChannelCounter2 = 0;
    let dataChannelCounter3 = 0;

    let onClientCallFinish = () => {
      clientCounter++;
      if (clientCounter >= 3) {
        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = () => {
      dataChannelCounter1++;
      if (dataChannelCounter1 >= 2) {
        expect(dataChannelClient1.isRtcConnected).to.eql(true);
        expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient2.id);
        expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient3.id);
        onClientCallFinish();
      }
    };
    dataChannelClient2.onDataChannelOpen = () => {
      dataChannelCounter2++;
      if (dataChannelCounter2 >= 2) {
        expect(dataChannelClient2.isRtcConnected).to.eql(true);
        expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient1.id);
        expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient3.id);
        onClientCallFinish();
      }
    };
    dataChannelClient3.onDataChannelOpen = () => {
      dataChannelCounter3++;
      if (dataChannelCounter3 >= 2) {
        expect(dataChannelClient3.isRtcConnected).to.eql(true);
        expect(dataChannelClient3.connectedRoomClientIds).to.include(dataChannelClient1.id);
        expect(dataChannelClient3.connectedRoomClientIds).to.include(dataChannelClient2.id);
        onClientCallFinish();
      }
    };

    dataChannelClient1.callAll();
  });

  it('callIds should call the specified clients', done => {
    let clientCounter = 0;

    let onClientCallFinish = () => {
      clientCounter++;
      if (clientCounter >= 2) {
        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = () => {
      expect(dataChannelClient1.isRtcConnected).to.eql(true);
      expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient2.id);
      onClientCallFinish();
    };
    dataChannelClient2.onDataChannelOpen = () => {
      expect(dataChannelClient2.isRtcConnected).to.eql(true);
      expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient1.id);
      onClientCallFinish();
    };
    dataChannelClient3.onDataChannelOpen = () => {
      expect.fail();
    };

    dataChannelClient1.callIds([dataChannelClient2.id]);
  });

  it('hangUpAll should hang up all clients', done => {
    let dataChannelOpenCounter = 0;
    let dataChannelCloseCounter = 0;

    let onDataChannelOpen = () => {
      dataChannelOpenCounter++;
      if (dataChannelOpenCounter >= 6) {
        dataChannelClient1.hangUpAll();
      }
    };

    let onDataChannelClose = () => {
      dataChannelCloseCounter++;
      if (dataChannelCloseCounter == 2) {
        dataChannelClient2.hangUpAll();
      }
      else if (dataChannelCloseCounter >= 3) {
        expect(dataChannelClient1.isRtcConnected).to.eql(false);
        expect(dataChannelClient2.isRtcConnected).to.eql(false);
        expect(dataChannelClient3.isRtcConnected).to.eql(false);

        expect(dataChannelClient1.connectedRoomClientIds).to.eql([]);
        expect(dataChannelClient2.connectedRoomClientIds).to.eql([]);
        expect(dataChannelClient3.connectedRoomClientIds).to.eql([]);

        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient2.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient3.onDataChannelOpen = onDataChannelOpen;

    dataChannelClient1.onDataChannelClose = onDataChannelClose;
    dataChannelClient2.onDataChannelClose = onDataChannelClose;
    dataChannelClient3.onDataChannelClose = onDataChannelClose;

    dataChannelClient1.callAll();
  });

  it('sendTo should send the data to the specified clients', done => {
    let dataChannelOpenCounter = 0;
    let dataChannelMessageCounter = 0;

    let onDataChannelOpen = () => {
      dataChannelOpenCounter++;
      if (dataChannelOpenCounter >= 6) {
        dataChannelClient1.sendTo('data1', [dataChannelClient2.id]);
        dataChannelClient2.sendTo('data2', [dataChannelClient3.id]);
        dataChannelClient3.sendTo('data3', [dataChannelClient1.id]);
      }
    };

    let onDataChannelMessageFinish = () => {
      dataChannelMessageCounter++;
      if (dataChannelMessageCounter >= 3) {
        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient2.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient3.onDataChannelOpen = onDataChannelOpen;

    dataChannelClient1.onDataChannelMessage = (id, name, data) => {
      expect(id).to.eql(dataChannelClient3.id);
      expect(name).to.eql('c3');
      expect(data).to.eql('data3');
      onDataChannelMessageFinish();
    };
    dataChannelClient2.onDataChannelMessage = (id, name, data) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(data).to.eql('data1');
      onDataChannelMessageFinish();
    };
    dataChannelClient3.onDataChannelMessage = (id, name, data) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(data).to.eql('data2');
      onDataChannelMessageFinish();
    };

    dataChannelClient1.callAll();
  });

  it('sendToAll should send the data to all clients', done => {
    let dataChannelOpenCounter = 0;
    let dataChannelMessageCounter = 0;

    let onDataChannelOpen = () => {
      dataChannelOpenCounter++;
      if (dataChannelOpenCounter >= 6) {
        dataChannelClient1.sendToAll('data1');
        dataChannelClient2.sendToAll('data2');
        dataChannelClient3.sendToAll('data3');
      }
    };

    let onDataChannelMessageFinish = () => {
      dataChannelMessageCounter++;
      if (dataChannelMessageCounter >= 6) {
        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient2.onDataChannelOpen = onDataChannelOpen;
    dataChannelClient3.onDataChannelOpen = onDataChannelOpen;

    dataChannelClient1.onDataChannelMessage = (id, name, data) => {
      if (id == dataChannelClient2.id) {
        expect(name).to.eql('c2');
        expect(data).to.eql('data2');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient3.id) {
        expect(name).to.eql('c3');
        expect(data).to.eql('data3');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };
    dataChannelClient2.onDataChannelMessage = (id, name, data) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(data).to.eql('data1');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient3.id) {
        expect(name).to.eql('c3');
        expect(data).to.eql('data3');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };
    dataChannelClient3.onDataChannelMessage = (id, name, data) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(data).to.eql('data1');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient2.id) {
        expect(name).to.eql('c2');
        expect(data).to.eql('data2');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };

    dataChannelClient1.callAll();
  });
});