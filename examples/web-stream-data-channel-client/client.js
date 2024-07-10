(function() {
  let localVideo = document.getElementById('local_video');
  let muteAudioButton = document.getElementById('mute_audio_button');
  let unmuteAudioButton = document.getElementById('unmute_audio_button');
  let muteVideoButton = document.getElementById('mute_video_button');
  let unmuteVideoButton = document.getElementById('unmute_video_button');
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
  let remoteVideos = document.getElementById('remote_videos');

  muteAudioButton.disabled = true;
  unmuteAudioButton.disabled = true;
  muteVideoButton.disabled = true;
  unmuteVideoButton.disabled = true;
  closeButton.disabled = true;
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  closeAllRoomPeerConnectionsButton.disabled = true;
  callOneButton.disabled = true;
  sendButton.disabled = true;

  let streamDataChannelClient = null;

  window.openteraWebrtcWebClient.devices.getDefaultStream().then(stream => {
    localVideo.srcObject = stream;
    localVideo.autoplay = true;
  });

  function connectStreamClientEvents() {
    streamDataChannelClient.onSignalingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    streamDataChannelClient.onSignalingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      closeAllRoomPeerConnectionsButton.disabled = true;
      callOneButton.disabled = true;
    };
    streamDataChannelClient.onSignalingConnectionError = message => {
      alert(message);
    }
    streamDataChannelClient.onRoomClientsChange = clients => {
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

    streamDataChannelClient.onClientConnectionFail = (id, name, clientData) => {
      console.log('The connect with the client ' + name + '(' + id + ') failed.');
    }

    streamDataChannelClient.onAddRemoteStream = (id, name, clientData, stream) => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;

      let h5 = document.createElement("h5");;
      h5.innerHTML = id + ' - ' + name;
      h5.id = 'h5' + id;

      let video = document.createElement("video");
      video.srcObject = stream;
      video.id = 'video' + id;
      video.autoplay = true;

      remoteVideos.appendChild(h5);
      remoteVideos.appendChild(video);
    }
    let onClientDisconnect = (id, name, clientData) => {
      sendButton.disabled = !streamDataChannelClient.isRtcConnected;
      callAllButton.disabled = streamDataChannelClient.isRtcConnected;
      hangUpAllButton.disabled = !streamDataChannelClient.isRtcConnected;
      closeAllRoomPeerConnectionsButton.disabled = !streamDataChannelClient.isRtcConnected;
      callOneButton.disabled = streamDataChannelClient.isRtcConnected;

      let h5 = document.getElementById('h5' + id);
      let video = document.getElementById('video' + id);
      if (h5 !== null) {
        remoteVideos.removeChild(document.getElementById('h5' + id));
      }
      if (video !== null) {
        remoteVideos.removeChild(document.getElementById('video' + id));
      }
    };
    streamDataChannelClient.onClientDisconnect = onClientDisconnect;

    streamDataChannelClient.onDataChannelOpen = (id, name, clientData) => {
      sendButton.disabled = false;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = false;
      closeAllRoomPeerConnectionsButton.disabled = false;
      callOneButton.disabled = true;
    }
    streamDataChannelClient.onDataChannelClose = onClientDisconnect;
    streamDataChannelClient.onDataChannelMessage = (id, name, clientData, message) => {
      chatTextArea.value += id + ' - ' + name + ': ';
      chatTextArea.value += message;
      chatTextArea.value += '\n';
    };
  }

  function updateMuteButtons() {
    muteAudioButton.disabled = streamDataChannelClient.isLocalAudioMuted;
    unmuteAudioButton.disabled = !streamDataChannelClient.isLocalAudioMuted;
    muteVideoButton.disabled = streamDataChannelClient.isLocalVideoMuted;
    unmuteVideoButton.disabled = !streamDataChannelClient.isLocalVideoMuted;
  }

  muteAudioButton.onclick = () => {
    streamDataChannelClient.muteLocalAudio();
    updateMuteButtons();
  };
  unmuteAudioButton.onclick = () => {
    streamDataChannelClient.unmuteLocalAudio();
    updateMuteButtons();
  };
  muteVideoButton.onclick = () => {
    streamDataChannelClient.muteLocalVideo();
    updateMuteButtons();
  };
  unmuteVideoButton.onclick = () => {
    streamDataChannelClient.unmuteLocalVideo();
    updateMuteButtons();
  };
  connectButton.onclick = async () => {
    const SignalingServerConfiguration = {
      url: 'ws://localhost:8080/signaling',
      name: nameInput.value,
      data: {}, // Client custom data
      room: 'chat',
      password: passwordInput.value
    };
    const StreamConfiguration = {
      localStream: localVideo.srcObject, // Optional
      isSendOnly: false
    };
    const DataChannelConfiguration = {}; // See: https://developer.mozilla.org/fr/docs/Web/API/RTCPeerConnection/createDataChannel#RTCDataChannelInit_dictionary
    const RtcConfiguration = { // See: https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/RTCPeerConnection#RTCConfiguration_dictionary
      iceServers: await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', passwordInput.value)
    };
    let logger = (...args) => console.log(...args);

    streamDataChannelClient = new window.openteraWebrtcWebClient.StreamDataChannelClient(SignalingServerConfiguration,
      StreamConfiguration, DataChannelConfiguration, RtcConfiguration, logger);
    connectStreamClientEvents();

    await streamDataChannelClient.connect();
    updateMuteButtons();
  };
  closeButton.onclick = () => {
    streamDataChannelClient.close();
    clientList.innerHTML = '';
    remoteVideos.innerHTML = '';
  };
  callAllButton.onclick = () => streamDataChannelClient.callAll();
  hangUpAllButton.onclick = () => streamDataChannelClient.hangUpAll();
  closeAllRoomPeerConnectionsButton.onclick = () => streamDataChannelClient.closeAllRoomPeerConnections();
  callOneButton.onclick = () => streamDataChannelClient.callIds([idInput.value]);
  sendButton.onclick = () => {
    chatTextArea.value += 'Me: ';
    chatTextArea.value += textInput.value;
    chatTextArea.value += '\n';

    streamDataChannelClient.sendToAll(textInput.value)
  };
})();
