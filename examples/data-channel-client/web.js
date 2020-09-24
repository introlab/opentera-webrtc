(function() {
  connectButton = document.getElementById('connect_button');
  closeButton = document.getElementById('close_button');
  callButton = document.getElementById('call_button');
  hangUpButton = document.getElementById('hang_up_button');
  textInput = document.getElementById('text_input');
  sendButton = document.getElementById('send_button');
  chatTextArea = document.getElementById('chat_text_area');

  closeButton.disabled = true;
  callButton.disabled = true;
  hangUpButton.disabled = true;
  sendButton.disabled = true;
  chatTextArea.value = '';

  wasConnected = false;


  signallingServerConfiguration = {
    url: 'http://localhost:8080'
  };
  dataChannelConfiguration = {
    name: 'chat'
  };
  rtcConfiguration = {};
  clientType = 'client';
  let dataChannelClient = new window.openteraWebrtcWebClient.DataChannelClient(signallingServerConfiguration, dataChannelConfiguration, rtcConfiguration, clientType);
  

  dataChannelClient.onSignallingConnectionOpen = () => {
    connectButton.disabled = true;
    closeButton.disabled = false;
  };
  dataChannelClient.onSignallingConnectionClose = async () => {
    connectButton.disabled = false;
    closeButton.disabled = true;
    callButton.disabled = true;
    hangUpButton.disabled = true;

    if (wasConnected) {
      window.setTimeout(async () => {
        await dataChannelClient.connect();
      }, 100);      
    }
  };
  dataChannelClient.onPeerReadyChanged = isReady => {
    callButton.disabled = !isReady;
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
  callButton.onclick = async () => await dataChannelClient.call();
  hangUpButton.onclick = () => {
    dataChannelClient.close();
  };
  sendButton.onclick = () => dataChannelClient.send(textInput.value);  
 })();
