describe('Right password DataChannelClient', done => {
  let dataChannelClient1;
  let dataChannelClient2;
  let dataChannelClient3;

  const BEFORE_AFTER_TIMEOUT_MS = 100;
  const TEST_TIMEOUT_MS = 15000;
  const SignalingServerConfiguration1 = {
    url: 'ws://localhost:8080/signaling',
    name: 'c1',
    data:'cd1',
    room: 'chat',
    password: 'abc'
  };
  const SignalingServerConfiguration2 = {
    url: 'ws://localhost:8080/signaling',
    name: 'c2',
    data:'cd2',
    room: 'chat',
    password: 'abc'
  };
  const SignalingServerConfiguration3 = {
    url: 'ws://localhost:8080/signaling',
    name: 'c3',
    data:'cd3',
    room: 'chat',
    password: 'abc'
  };
  const DataChannelConfiguration = {};
  const RtcConfiguration = {};

  beforeEach(done => {
    let counter = 0;
    dataChannelClient1 = new window.openteraWebrtcWebClient.DataChannelClient(SignalingServerConfiguration1,
      DataChannelConfiguration, RtcConfiguration);
    dataChannelClient2 = new window.openteraWebrtcWebClient.DataChannelClient(SignalingServerConfiguration2,
      DataChannelConfiguration, RtcConfiguration);
    dataChannelClient3 = new window.openteraWebrtcWebClient.DataChannelClient(SignalingServerConfiguration3,
      DataChannelConfiguration, RtcConfiguration);

    let onSignalingConnectionOpen = () => {
      counter++;
      if (counter >= 3) {
        setTimeout(done, BEFORE_AFTER_TIMEOUT_MS);
      }
    };
    let onSignalingConnectionError = () => {
      expect.fail();
      done();
    };
    dataChannelClient1.onSignalingConnectionOpen = onSignalingConnectionOpen;
    dataChannelClient1.onSignalingConnectionError = onSignalingConnectionError;
    dataChannelClient2.onSignalingConnectionOpen = onSignalingConnectionOpen;
    dataChannelClient2.onSignalingConnectionError = onSignalingConnectionError;
    dataChannelClient3.onSignalingConnectionOpen = onSignalingConnectionOpen;
    dataChannelClient3.onSignalingConnectionError = onSignalingConnectionError;

    dataChannelClient1.connect();
    setTimeout(() => dataChannelClient2.connect(), BEFORE_AFTER_TIMEOUT_MS);
    setTimeout(() => dataChannelClient3.connect(), 2 * BEFORE_AFTER_TIMEOUT_MS);
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
  }).timeout(TEST_TIMEOUT_MS);
  it('isRtcConnected should return false', () => {
    expect(dataChannelClient1.isRtcConnected).to.eql(false);
    expect(dataChannelClient2.isRtcConnected).to.eql(false);
    expect(dataChannelClient3.isRtcConnected).to.eql(false);
  }).timeout(TEST_TIMEOUT_MS);
  it('id should not return null', () => {
    expect(dataChannelClient1.id).to.not.eql(null);
    expect(dataChannelClient2.id).to.not.eql(null);
    expect(dataChannelClient3.id).to.not.eql(null);
  }).timeout(TEST_TIMEOUT_MS);
  it('connectedRoomClientIds should return []', () => {
    expect(dataChannelClient1.connectedRoomClientIds).to.eql([]);
    expect(dataChannelClient2.connectedRoomClientIds).to.eql([]);
    expect(dataChannelClient3.connectedRoomClientIds).to.eql([]);
  }).timeout(TEST_TIMEOUT_MS);
  it('roomClients should return all clients', () => {
    expect(dataChannelClient1.roomClients.length).to.eql(3);
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', data: 'cd1', isConnected: true});
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', data: 'cd2', isConnected: false});
    expect(dataChannelClient1.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', data: 'cd3', isConnected: false});

    expect(dataChannelClient2.roomClients.length).to.eql(3);
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', data: 'cd1', isConnected: false});
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', data: 'cd2', isConnected: true});
    expect(dataChannelClient2.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', data: 'cd3', isConnected: false});

    expect(dataChannelClient3.roomClients.length).to.eql(3);
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient1.id, name: 'c1', data: 'cd1', isConnected: false});
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient2.id, name: 'c2', data: 'cd2', isConnected: false});
    expect(dataChannelClient3.roomClients).to.include({ id: dataChannelClient3.id, name: 'c3', data: 'cd3', isConnected: true});
  }).timeout(TEST_TIMEOUT_MS);

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
        expect(dataChannelClient1.connectedRoomClientIds.length).to.eql(2);
        expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient2.id);
        expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient3.id);
        onClientCallFinish();
      }
    };
    dataChannelClient2.onDataChannelOpen = () => {
      dataChannelCounter2++;
      if (dataChannelCounter2 >= 2) {
        expect(dataChannelClient2.isRtcConnected).to.eql(true);
        expect(dataChannelClient2.connectedRoomClientIds.length).to.eql(2);
        expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient1.id);
        expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient3.id);
        onClientCallFinish();
      }
    };
    dataChannelClient3.onDataChannelOpen = () => {
      dataChannelCounter3++;
      if (dataChannelCounter3 >= 2) {
        expect(dataChannelClient3.isRtcConnected).to.eql(true);
        expect(dataChannelClient3.connectedRoomClientIds.length).to.eql(2);
        expect(dataChannelClient3.connectedRoomClientIds).to.include(dataChannelClient1.id);
        expect(dataChannelClient3.connectedRoomClientIds).to.include(dataChannelClient2.id);
        onClientCallFinish();
      }
    };

    dataChannelClient1.callAll();
  }).timeout(TEST_TIMEOUT_MS);

  it('callIds should call the specified clients', done => {
    let clientCounter = 0;

    let onClientCallFinish = () => {
      clientCounter++;
      if (clientCounter >= 2) {
        expect(dataChannelClient3.connectedRoomClientIds).to.eql([]);
        done();
      }
    };

    dataChannelClient1.onDataChannelOpen = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(clientData).to.eql('cd2');

      expect(dataChannelClient1.isRtcConnected).to.eql(true);
      expect(dataChannelClient1.connectedRoomClientIds.length).to.eql(1);
      expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient2.id);
      onClientCallFinish();
    };
    dataChannelClient2.onDataChannelOpen = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(clientData).to.eql('cd1');

      expect(dataChannelClient2.isRtcConnected).to.eql(true);
      expect(dataChannelClient2.connectedRoomClientIds.length).to.eql(1);
      expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient1.id);
      onClientCallFinish();
    };
    dataChannelClient3.onDataChannelOpen = () => {
      expect.fail();
    };

    dataChannelClient1.callIds([dataChannelClient2.id]);
  }).timeout(TEST_TIMEOUT_MS);

  it('onClientConnect should be called after a call', done => {
    let clientCounter = 0;

    let onClientCallFinish = () => {
      clientCounter++;
      if (clientCounter >= 2) {
        done();
      }
    };

    dataChannelClient1.onClientConnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(clientData).to.eql('cd2');

      onClientCallFinish();
    };
    dataChannelClient2.onClientConnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(clientData).to.eql('cd1');

      onClientCallFinish();
    };
    dataChannelClient3.onClientConnect = () => {
      expect.fail();
    };

    dataChannelClient1.callIds([dataChannelClient2.id]);
  }).timeout(TEST_TIMEOUT_MS);

  it('onClientDisconnect should be called after the hangUpAll function call', done => {
    let clientCounter = 0;

    let onClientCallFinish = () => {
      clientCounter++;
      if (clientCounter >= 2) {
        done();
      }
    };

    dataChannelClient1.onClientConnect = () => {
      dataChannelClient1.hangUpAll();
    };

    dataChannelClient1.onClientDisconnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(clientData).to.eql('cd2');

      onClientCallFinish();
    };
    dataChannelClient2.onClientDisconnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(clientData).to.eql('cd1');

      onClientCallFinish();
    };
    dataChannelClient3.onClientConnect = () => {
      expect.fail();
    };

    dataChannelClient1.callIds([dataChannelClient2.id]);
  }).timeout(TEST_TIMEOUT_MS);

  it('callAcceptor should be able to reject a call and onCallReject should be called', done => {
    let clientCallFinishCounter = 0;
    let clientCallRejectCounter = 0;

    let onFinish = () => {
      if (clientCallFinishCounter >= 2 && clientCallRejectCounter >= 2) {
        expect(dataChannelClient1.isRtcConnected).to.eql(true);
        expect(dataChannelClient1.connectedRoomClientIds.length).to.eql(1);
        expect(dataChannelClient1.connectedRoomClientIds).to.include(dataChannelClient2.id);

        expect(dataChannelClient2.isRtcConnected).to.eql(true);
        expect(dataChannelClient2.connectedRoomClientIds.length).to.eql(1);
        expect(dataChannelClient2.connectedRoomClientIds).to.include(dataChannelClient1.id);

        expect(dataChannelClient3.isRtcConnected).to.eql(false);
        expect(dataChannelClient3.connectedRoomClientIds).to.eql([]);

        done();
      }
    }

    let onClientCallFinish = () => {
      clientCallFinishCounter++;
      onFinish();
    };

    let onClientCallRejectFinish = () => {
      clientCallRejectCounter++;
      onFinish();
    }

    dataChannelClient1.onClientConnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(clientData).to.eql('cd2');

      onClientCallFinish();
    };
    dataChannelClient2.onClientConnect = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(clientData).to.eql('cd1');

      onClientCallFinish();
    };
    dataChannelClient3.onClientConnect = () => {
      expect.fail();
    };

    dataChannelClient1.callAcceptor = async (id, name, clientData) => {
      expect.fail();
      return true;
    };

    dataChannelClient2.callAcceptor = (id, name, clientData) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(clientData).to.eql('cd1');
      }
      else if (id == dataChannelClient3.id) {
        expect(name).to.eql('c3');
        expect(clientData).to.eql('cd3');
      }
      else {
        expect.fail();
      }

      return id == dataChannelClient1.id;
    };

    dataChannelClient3.callAcceptor = (id, name, clientData) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(clientData).to.eql('cd1');
      }
      else if (id == dataChannelClient2.id) {
        expect(name).to.eql('c2');
        expect(clientData).to.eql('cd2');
      }
      else {
        expect.fail();
      }

      return id == dataChannelClient2.id;
    };

    dataChannelClient1.onCallReject = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient3.id);
      expect(name).to.eql('c3');
      expect(clientData).to.eql('cd3');
      onClientCallRejectFinish();
    };

    dataChannelClient2.onCallReject = (id, name, clientData) => {
      expect(id).to.eql(dataChannelClient3.id);
      expect(name).to.eql('c3');
      expect(clientData).to.eql('cd3');
      onClientCallRejectFinish();
    };

    dataChannelClient3.onCallReject = () => {
      expect.fail();
    };

    dataChannelClient1.callAll();
  }).timeout(TEST_TIMEOUT_MS);

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
      if (dataChannelCloseCounter == 4) {
        dataChannelClient2.hangUpAll();
      }
      else if (dataChannelCloseCounter >= 6) {
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
  }).timeout(TEST_TIMEOUT_MS);

  it('closeAllRoomPeerConnections should close all room peer connections', done => {
    let dataChannelOpenCounter = 0;
    let dataChannelCloseCounter = 0;

    let onDataChannelOpen = () => {
      dataChannelOpenCounter++;
      if (dataChannelOpenCounter >= 6) {
        dataChannelClient1.closeAllRoomPeerConnections();
      }
    };

    let onDataChannelClose = () => {
      dataChannelCloseCounter++;
      if (dataChannelCloseCounter >= 6) {
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
  }).timeout(TEST_TIMEOUT_MS);

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

    dataChannelClient1.onDataChannelMessage = (id, name, clientData, data) => {
      expect(id).to.eql(dataChannelClient3.id);
      expect(name).to.eql('c3');
      expect(clientData).to.eql('cd3');
      expect(data).to.eql('data3');
      onDataChannelMessageFinish();
    };
    dataChannelClient2.onDataChannelMessage = (id, name, clientData, data) => {
      expect(id).to.eql(dataChannelClient1.id);
      expect(name).to.eql('c1');
      expect(clientData).to.eql('cd1');
      expect(data).to.eql('data1');
      onDataChannelMessageFinish();
    };
    dataChannelClient3.onDataChannelMessage = (id, name, clientData, data) => {
      expect(id).to.eql(dataChannelClient2.id);
      expect(name).to.eql('c2');
      expect(clientData).to.eql('cd2');
      expect(data).to.eql('data2');
      onDataChannelMessageFinish();
    };

    dataChannelClient1.callAll();
  }).timeout(TEST_TIMEOUT_MS);

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

    dataChannelClient1.onDataChannelMessage = (id, name, clientData, data) => {
      if (id == dataChannelClient2.id) {
        expect(name).to.eql('c2');
        expect(clientData).to.eql('cd2');
        expect(data).to.eql('data2');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient3.id) {
        expect(name).to.eql('c3');
        expect(clientData).to.eql('cd3');
        expect(data).to.eql('data3');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };
    dataChannelClient2.onDataChannelMessage = (id, name, clientData, data) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(clientData).to.eql('cd1');
        expect(data).to.eql('data1');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient3.id) {
        expect(name).to.eql('c3');
        expect(clientData).to.eql('cd3');
        expect(data).to.eql('data3');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };
    dataChannelClient3.onDataChannelMessage = (id, name, clientData, data) => {
      if (id == dataChannelClient1.id) {
        expect(name).to.eql('c1');
        expect(clientData).to.eql('cd1');
        expect(data).to.eql('data1');
        onDataChannelMessageFinish();
      }
      else if (id == dataChannelClient2.id) {
        expect(name).to.eql('c2');
        expect(clientData).to.eql('cd2');
        expect(data).to.eql('data2');
        onDataChannelMessageFinish();
      }
      else {
        expect.fail();
      }
    };

    dataChannelClient1.callAll();
  }).timeout(TEST_TIMEOUT_MS);
});
