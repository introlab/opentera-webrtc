#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

#include <utility>

using namespace introlab;
using namespace std;

/**
 * Construct a video stream peer connection handler
 * @param id this peer id
 * @param peerClient this peer client object
 * @param isCaller did this peer initiated the call
 * @param sendEvent function to send event on the signaling connection
 * @param onError function executed on signaling connection error
 * @param onClientConnected function executed when the signaling connection is opened
 * @param onClientDisconnected function executed when the signaling connection is closed
 * @param videoTrack the video track to add to the call
 */
VideoStreamPeerConnectionHandler::VideoStreamPeerConnectionHandler(
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

/**
 * @brief sets the peer connection that needs to be handled
 *
 * We add the video track to the peer connection then pass the connection to the base class
 *
 * @param peerConnection the peer connection to handle
 */
void VideoStreamPeerConnectionHandler::setPeerConnection(
        const rtc::scoped_refptr<webrtc::PeerConnectionInterface>& peerConnection)
{
    peerConnection->AddTrack(m_videoTrack, vector<string>());
    PeerConnectionHandler::setPeerConnection(peerConnection);
}
