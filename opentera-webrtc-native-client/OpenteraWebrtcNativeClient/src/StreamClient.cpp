#include <OpenteraWebrtcNativeClient/StreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/StreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

/**
 * @brief construct a video stream client
 *
 * @param signallingServerConfiguration configuration to connect to the signaling server
 * @param webrtcConfiguration webrtc configuration
 * @param videoSource the video source that this client will add to the call
 */
StreamClient::StreamClient(
        const SignallingServerConfiguration& signallingServerConfiguration,
        const WebrtcConfiguration& webrtcConfiguration,
        const shared_ptr<VideoSource>& videoSource) :
        SignallingClient(signallingServerConfiguration, webrtcConfiguration),
        m_videoSource(videoSource)
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
unique_ptr<PeerConnectionHandler> StreamClient::createPeerConnectionHandler(const string& id,
                                                                            const Client& peerClient, bool isCaller)
{
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack =
            m_peerConnectionFactory->CreateVideoTrack("stream", m_videoSource.get());

    return make_unique<StreamPeerConnectionHandler>(
            id,
            peerClient,
            isCaller,
            getSendEventFunction(),
            getOnErrorFunction(),
            getOnClientConnectedFunction(),
            getOnClientDisconnectedFunction(),
            videoTrack);
}
