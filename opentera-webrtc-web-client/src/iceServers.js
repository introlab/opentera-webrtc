/**
 * @brief Gets the ice servers from the signaling server.
 * @param {String} url The signaling server ice servers address.
 * @param {String} password The signaling server password.
 * @returns {Promise<Object>} THe signaling server ice servers
 */
async function fetchFromServer(url, password) {
  let response = await fetch(url, {
    method: 'GET',
    headers: new Headers({ 'Authorization': password })
  });

  return await response.json();
}

let iceServers = {
  fetchFromServer
};

export default iceServers;
