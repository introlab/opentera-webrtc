(function() {
  connectButton = document.getElementById('connect_button');
  closeButton = document.getElementById('close_button');
  textInput = document.getElementById('text_input');
  sendButton = document.getElementById('send_button');
  chatTextArea = document.getElementById('chat_text_area');

  closeButton.disabled = true;
  sendButton.disabled = true;
  chatTextArea.value = '';

  wasConnected = false;


  signallingServerConfiguration = {
    url: 'http://localhost:8080'
  };
  dataChannelConfiguration = {
    name: 'chat',
    isRemote: true
  };
  rtcConfiguration = {};
  clientType = 'robot';
  let dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration, clientType);
  

  dataChannelClient.onSignallingConnectionOpen = () => {
    connectButton.disabled = true;
    closeButton.disabled = false;
  };
  dataChannelClient.onSignallingConnectionClose = async () => {
    connectButton.disabled = false;
    closeButton.disabled = true;

    if (wasConnected) {
      window.setTimeout(async () => {
        await dataChannelClient.connect();
      }, 100);      
    }
  };

  dataChannelClient.onDataChannelOpen = () => sendButton.disabled = false;
  dataChannelClient.onDataChannelClose = () => sendButton.disabled = true;
  dataChannelClient.onDataChannelMessage = message => {
    chatTextArea.value += message;
    chatTextArea.value += '\n';
  };


  connectButton.onclick = async () => {
    wasConnected = true;
    await dataChannelClient.connect();
  };
  closeButton.onclick = () => {
    wasConnected = false;
    dataChannelClient.close();
  };
  sendButton.onclick = () => {
    dataChannelClient.send(textInput.value);
  }
 })();
