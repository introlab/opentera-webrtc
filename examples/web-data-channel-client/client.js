(function() {
  let nameInput = document.getElementById('name_input');
  let passwordInput = document.getElementById('password_input');
  let connectButton = document.getElementById('connect_button');
  let closeButton = document.getElementById('close_button');
  let clientList = document.getElementById('client_list');
  let callAllButton = document.getElementById('call_all_button');
  let hangUpAllButton = document.getElementById('hang_up_all_button');
  let closeAllRoomPeerConnectionsButton = document.getElementById('close_all_room_peer_connections');
  let idInput = document.getElementById('id_input');
  let callOneButton = document.getElementById('call_one_button');
  let textInput = document.getElementById('text_input');
  let sendButton = document.getElementById('send_button');
  let chatTextArea = document.getElementById('chat_text_area');

  closeButton.disabled = true;
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  closeAllRoomPeerConnectionsButton.disabled = true;
  callOneButton.disabled = true;
  sendButton.disabled = true;
  chatTextArea.value = '';

  let dataChannelClient = null;

  function connectDataChannelClientEvents() {
    dataChannelClient.onSignalingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    dataChannelClient.onSignalingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      closeAllRoomPeerConnectionsButton.disabled = true;
      callOneButton.disabled = true;
    };
    dataChannelClient.onSignalingConnectionError = message => {
      alert(message);
    }
    dataChannelClient.onRoomClientsChange = clients => {
      callAllButton.disabled = !(clients.length > 1 && hangUpAllButton.disabled);
      callOneButton.disabled = callAllButton.disabled;

      clientList.innerHTML = '';
      clients.forEach(client => {
        let li = document.createElement('li');
        li.textContent = client.id + ' - ' + client.name;
        li.style.color = client.isConnected ? 'green' : 'red';
        clientList.appendChild(li);
      });
    };

    dataChannelClient.callAcceptor = async (id, name, clientData) => {
      return confirm('Do you accept the call from ' + name + '?');
    };

    dataChannelClient.onCallReject = (id, name, clientData) => {
      alert('The call is rejected (' + name + ')');
    };

    dataChannelClient.onClientConnectionFail = (id, name, clientData) => {
      console.log('The connect with the client ' + name + '(' + id + ') failed.');
    }

    dataChannelClient.onDataChannelOpen = (id, name, clientData) => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;
    }
    dataChannelClient.onDataChannelClose = (id, name, clientData) => {
      sendButton.disabled = !dataChannelClient.isRtcConnected;
      callAllButton.disabled = dataChannelClient.isRtcConnected;
      hangUpAllButton.disabled = !dataChannelClient.isRtcConnected;
      closeAllRoomPeerConnectionsButton.disabled = !dataChannelClient.isRtcConnected;
      callOneButton.disabled = dataChannelClient.isRtcConnected;
    };
    dataChannelClient.onDataChannelMessage = (id, name, clientData, message) => {
      chatTextArea.value += id + ' - ' + name + ': ';
      chatTextArea.value += message;
      chatTextArea.value += '\n';
    };
  }


  connectButton.onclick = async () => {
    const SignalingServerConfiguration = {
      url: 'ws://localhost:8080/signaling',
      name: nameInput.value,
      data: {}, // Client custom data
      room: 'chat',
      password: passwordInput.value
    };
    const DataChannelConfiguration = {}; // See: https://developer.mozilla.org/fr/docs/Web/API/RTCPeerConnection/createDataChannel#RTCDataChannelInit_dictionary
    const RtcConfiguration = { // See: https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/RTCPeerConnection#RTCConfiguration_dictionary
      iceServers: await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', passwordInput.value)
    };
    let logger = (...args) => console.log(...args);

    dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignalingServerConfiguration,
      DataChannelConfiguration, RtcConfiguration, logger);
    connectDataChannelClientEvents();

    await dataChannelClient.connect();
  };
  closeButton.onclick = () => {
    dataChannelClient.close();
    clientList.innerHTML = '';
  };
  callAllButton.onclick = () => dataChannelClient.callAll();
  hangUpAllButton.onclick = () => dataChannelClient.hangUpAll();
  closeAllRoomPeerConnectionsButton.onclick = () => dataChannelClient.closeAllRoomPeerConnections();
  callOneButton.onclick = () => dataChannelClient.callIds([idInput.value]);
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ';
    chatTextArea.value += textInput.value;
    chatTextArea.value += '\n';

    dataChannelClient.sendToAll(textInput.value)
  };
})();
