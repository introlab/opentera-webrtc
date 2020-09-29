describe('iceServers', () => {
  it('should get an empty array if the password is wrong', async () => {
    let iceServers = await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', '');
    expect(iceServers).to.eql([]);
  });
  it('should get an the array if the password is right', async () => {
    let iceServers = await window.openteraWebrtcWebClient.iceServers.fetchFromServer('http://localhost:8080/iceservers', 'abc');
    expect(iceServers).to.eql([{ urls: "stun:stun.l.google.com:19302" }]);
  });
});
