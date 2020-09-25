
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
