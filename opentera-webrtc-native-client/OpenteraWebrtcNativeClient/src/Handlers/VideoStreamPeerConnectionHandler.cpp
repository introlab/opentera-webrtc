#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

#include <utility>

using namespace introlab;
using namespace std;

VideoStreamPeerConnectionHandler::VideoStreamPeerConnectionHandler(
    string id,
    Client peerClient,
    bool isCaller,
    function<void(const string&, const sio::message::ptr&)> sendEvent,
    function<void(const string&)> onError,
    function<void(const Client&)> onClientConnected,
    function<void(const Client&)> onClientDisconnected,
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack) :
    PeerConnectionHandler(move(id), move(peerClient), isCaller, move(sendEvent), move(onError), move(onClientConnected),
            move(onClientDisconnected)),
    m_videoTrack(move(videoTrack))
{
}

void VideoStreamPeerConnectionHandler::setPeerConnection(
        const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    peerConnection->AddTrack(m_videoTrack, vector<string>());
    PeerConnectionHandler::setPeerConnection(peerConnection);
}
