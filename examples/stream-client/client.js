(function() {
  let localVideo = document.getElementById('local_video');
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
  let remoteVideos = document.getElementById('remote_videos');

  closeButton.disabled = true;
  callAllButton.disabled = true;
  hangUpAllButton.disabled = true;
  closeAllRoomPeerConnectionsButton.disabled = true;
  callOneButton.disabled = true;

  let streamClient = null;

  window.openteraWebrtcWebClient.devices.getDefaultStream().then(stream => {
    localVideo.srcObject = stream;
    localVideo.autoplay = true;
  });

  function connectStreamClientEvents() {
    streamClient.onSignallingConnectionOpen = () => {
      connectButton.disabled = true;
      closeButton.disabled = false;
    };
    streamClient.onSignallingConnectionClose = async () => {
      connectButton.disabled = false;
      closeButton.disabled = true;
      callAllButton.disabled = true;
      hangUpAllButton.disabled = true;
      closeAllRoomPeerConnectionsButton.disabled = true;
      callOneButton.disabled = true;
    };
    streamClient.onSignallingConnectionError = message => {
      alert(message);
    }
    streamClient.onRoomClientsChanged = clients => {
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

    streamClient.onAddRemoteStream = (id, name, clientData, stream) => {
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
    streamClient.onClientDisconnect = (id, name, clientData) => {
      callAllButton.disabled = streamClient.isRtcConnected;
      hangUpAllButton.disabled = !streamClient.isRtcConnected;
      closeAllRoomPeerConnectionsButton.disabled = !streamClient.isRtcConnected;
      callOneButton.disabled = streamClient.isRtcConnected;

      remoteVideos.removeChild(document.getElementById('h5' + id));
      remoteVideos.removeChild(document.getElementById('video' + id));
    };
  }


  connectButton.onclick = async () => {
    const SignallingServerConfiguration = {
      url: 'http://localhost:8080',
      name: nameInput.value,
      data: {}, // Client custom data
      room: 'chat',
      password: passwordInput.value
    };
    const StreamConfiguration = {
      localStream: localVideo.srcObject, // Optional
      isSendOnly: false
    };
    const RtcConfiguration = { // See: https://developer.mozilla.org/en-US/docs/Web/API/RTCPeerConnection/RTCPeerConnection#RTCConfiguration_dictionary
      iceServers: await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', passwordInput.value)
    };
    let logger = (...args) => console.log(...args);

    streamClient = new window.openteraWebrtcWebClient.StreamClient(SignallingServerConfiguration,
      StreamConfiguration, RtcConfiguration, logger);
    connectStreamClientEvents();

    await streamClient.connect();
  };
  closeButton.onclick = () => {
    streamClient.close();
    clientList.innerHTML = '';
    remoteVideos.innerHTML = '';
  };
  callAllButton.onclick = () => streamClient.callAll();
  hangUpAllButton.onclick = () => streamClient.hangUpAll();
  closeAllRoomPeerConnectionsButton.onclick = () => streamClient.closeAllRoomPeerConnections();
  callOneButton.onclick = () => streamClient.callIds([idInput.value]);
})();
