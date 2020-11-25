#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

#include <utility>

using namespace introlab;
using namespace std;

StreamPeerConnectionHandler::StreamPeerConnectionHandler(
    const string& id,
    const Client& peerClient,
    bool isCaller,
    const function<void(const string&, sio::message::ptr)>& sendEvent,
    const function<void(const string&)>& onError,
    const function<void(const Client&)>& onClientConnected,
    const function<void(const Client&)>& onClientDisconnected,
    rtc::scoped_refptr<webrtc::VideoTrackInterface>  videoTrack) :
    PeerConnectionHandler(id, peerClient, isCaller, sendEvent, onError, onClientConnected, onClientDisconnected),
    m_videoTrack(std::move(videoTrack))
{

}

void StreamPeerConnectionHandler::setPeerConnection(
        const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    peerConnection->AddTrack(m_videoTrack, vector<string>());
    PeerConnectionHandler::setPeerConnection(peerConnection);
}
