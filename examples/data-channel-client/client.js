(function() {
  let nameInput = document.getElementById('name_input');
  let passwordInput = document.getElementById('password_input');
  let connectButton = document.getElementById('connect_button');
  let closeButton = document.getElementById('close_button');
  let clientList = document.getElementById('client_list');
  let callAllButton = document.getElementById('call_all_button');
  let hangUpAllButton = document.getElementById('hang_up_all_button');
  let idInput = document.getElementById('id_input');
  let callOneButton = document.getElementById('call_one_button');
  let textInput = document.getElementById('text_input');
  let sendButton = document.getElementById('send_button');
  let chatTextArea = document.getElementById('chat_text_area');

  closeButton.disabled = true;
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  callOneButton.disabled = true;
  sendButton.disabled = true;
  chatTextArea.value = '';

  let dataChannelClient = null; 

  function connectDataChannelClientEvents() {
    dataChannelClient.onSignallingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    dataChannelClient.onSignallingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      callOneButton.disabled = true;
    };
    dataChannelClient.onSignallingConnectionError = message => {
      alert(message);
    }
    dataChannelClient.onRoomClientsChanged = clients => {
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

    dataChannelClient.onDataChannelOpen = () => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      callOneButton.disabled = true;
    }
    dataChannelClient.onDataChannelClose = () => {
      sendButton.disabled = !dataChannelClient.isRtcConnected;
      callAllButton.disabled = dataChannelClient.isRtcConnected;
      hangUpAllButton.disabled = !dataChannelClient.isRtcConnected;
      callOneButton.disabled = dataChannelClient.isRtcConnected;
    };
    dataChannelClient.onDataChannelMessage = (id, name, message) => {
      chatTextArea.value += id + ' - ' + name + ': ';
      chatTextArea.value += message;
      chatTextArea.value += '\n';
    };
  }


  connectButton.onclick = async () => {
    const SignallingServerConfiguration = {
      url: 'http://localhost:8080',
      name: nameInput.value,
      room: 'chat',
      password: passwordInput.value
    };
    const DataChannelConfiguration = {};
    const RtcConfiguration = {
      iceServers: await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', passwordInput.value)
    };

    dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration,
      DataChannelConfiguration, RtcConfiguration);
    connectDataChannelClientEvents();

    await dataChannelClient.connect();
  };
  closeButton.onclick = () => {
    dataChannelClient.close();
    clientList.innerHTML = '';
  };
  callAllButton.onclick = () => dataChannelClient.callAll();
  hangUpAllButton.onclick = () => {
    dataChannelClient.hangUpAll();
    hangUpAllButton.disabled = true;
    callAllButton.disabled = false;
    sendButton.disabled = true;
    callOneButton.disabled = false;
  };
  callOneButton.onclick = () => dataChannelClient.callIds([idInput.value]);
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ';
    chatTextArea.value += textInput.value;
    chatTextArea.value += '\n';

    dataChannelClient.sendToAll(textInput.value)
  };
})();
