(function() {
  let nameInput = document.getElementById('name_input');
  let passwordInput = document.getElementById('password_input');
  let connectButton = document.getElementById('connect_button');
  let closeButton = document.getElementById('close_button');
  let clientList = document.getElementById('client_list');
  let callButton = document.getElementById('call_button');
  let hangUpButton = document.getElementById('hang_up_button');
  let textInput = document.getElementById('text_input');
  let sendButton = document.getElementById('send_button');
  let chatTextArea = document.getElementById('chat_text_area');

  closeButton.disabled = true;
  callButton.disabled = true;
  hangUpButton.disabled = true;
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
      callButton.disabled = true;
      hangUpButton.disabled = true;
    };
    dataChannelClient.onSignallingConnectionError = message => {
      alert(message);
    }
    dataChannelClient.onRoomClientsChanged = clients => {
      callButton.disabled = !(clients.length > 1 && hangUpButton.disabled);

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
      callButton.disabled = true;
      hangUpButton.disabled = false;
    }
    dataChannelClient.onDataChannelClose = () => {
      sendButton.disabled = !dataChannelClient.isConnected;
      callButton.disabled = dataChannelClient.isConnected;
      hangUpButton.disabled = !dataChannelClient.isConnected;
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
    const RtcConfiguration = {};

    dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration,
      DataChannelConfiguration, RtcConfiguration);
    connectDataChannelClientEvents();

    await dataChannelClient.connect();
  };
  closeButton.onclick = () => {
    dataChannelClient.close();
    clientList.innerHTML = '';
  };
  callButton.onclick = async () => await dataChannelClient.callAll();
  hangUpButton.onclick = () => {
    dataChannelClient.hangUp();
    hangUpButton.disabled = true;
    callButton.disabled = false;
    sendButton.disabled = true;
  };
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ';
    chatTextArea.value += textInput.value;
    chatTextArea.value += '\n';

    dataChannelClient.sendToAll(textInput.value)
  };
})();
