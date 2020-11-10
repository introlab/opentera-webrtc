#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

VideoStreamPeerConnectionHandler::VideoStreamPeerConnectionHandler(const std::string& id,
    const Client& peerClient,
    bool isCaller,
    const std::function<void(const std::string&, sio::message::ptr)>& sendEvent,
    const std::function<void(const std::string&)>& onError,
    const std::function<void(const Client&)>& onClientConnected,
    const std::function<void(const Client&)>& onClientDisconnected,
    const rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack) :
    PeerConnectionHandler(id, peerClient, isCaller, sendEvent, onError, onClientConnected, onClientDisconnected),
    m_videoTrack(videoTrack)
{

}

VideoStreamPeerConnectionHandler::~VideoStreamPeerConnectionHandler()
{

}

void VideoStreamPeerConnectionHandler::setPeerConnection(
        const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    peerConnection->AddTrack(m_videoTrack,
    PeerConnectionHandler::setPeerConnection(peerConnection);
}
