#include <OpenteraWebrtcNativeClient/VideoStreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param videoSource the video source that this client will add to the call
 */
VideoStreamClient::VideoStreamClient(
        SignallingServerConfiguration signallingServerConfiguration,
        WebrtcConfiguration webrtcConfiguration,
        shared_ptr<VideoSource> videoSource) :
        SignallingClient(move(signallingServerConfiguration), move(webrtcConfiguration)),
        m_videoSource(move(videoSource))
{

}

/**
 * @brief Create the peer connection handler for this client
 *
 * @param id this peer id
 * @param peerClient this peer client object
 * @param isCaller indicate if this peer initiated the call
 * @return the peer connection handler
 */
unique_ptr<PeerConnectionHandler> VideoStreamClient::createPeerConnectionHandler(const string& id,
        const Client& peerClient, bool isCaller)
{
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack =
            m_peerConnectionFactory->CreateVideoTrack("stream", m_videoSource.get());

    return make_unique<VideoStreamPeerConnectionHandler>(
            id,
            peerClient,
            isCaller,
            getSendEventFunction(),
            getOnErrorFunction(),
            getOnClientConnectedFunction(),
            getOnClientDisconnectedFunction(),
            videoTrack);
}
