(function() {
  let nameInput = document.getElementById('name_input');
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
    dataChannelClient.onRoomClientsChanged = clients => {
      if (clients.length > 1 && hangUpButton.disabled) {
        callButton.disabled = false;
      }
      else {
        callButton.disabled = true;
      }

      clientList.innerHTML = '';
      clients.forEach(client => {
        let li = document.createElement('li');
        li.textContent = client.id + ' - ' + client.name;
        clientList.appendChild(li);
      });
    };

    dataChannelClient.onDataChannelOpen = () => {
      sendButton.disabled = false;
      callButton.disabled = true;
      hangUpButton.disabled = false;
    }
    dataChannelClient.onDataChannelClose = () => {
      sendButton.disabled = true;
      callButton.disabled = !dataChannelClient.isReady;
      hangUpButton.disabled = true;
    };
    dataChannelClient.onDataChannelMessage = (id, name, message) => {
      chatTextArea.value += id + ' - ' + name + ': ';
      chatTextArea.value += message;
      chatTextArea.value += '\n';
    };
  }


  connectButton.onclick = async () => {
    const SignallingServerConfiguration = {
      url: 'http://localhost:8080'
    };
    const DataChannelConfiguration = {};
    const RtcConfiguration = {};
    const Room = 'chat';

    dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(SignallingServerConfiguration, 
      DataChannelConfiguration, RtcConfiguration, nameInput.value, Room);
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
  };
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ';
    chatTextArea.value += textInput.value;
    chatTextArea.value += '\n';

    dataChannelClient.sendToAll(textInput.value)
  };  
})();
