#include <OpenteraWebrtcNativeClient/VideoStreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;
using namespace rtc;
using namespace webrtc;

VideoStreamClient::VideoStreamClient(
        const SignallingServerConfiguration& signallingServerConfiguration,
        const WebrtcConfiguration& webrtcConfiguration,
        const rtc::scoped_refptr<rtc::RefCountedObject<rtc::AdaptedVideoTrackSource>> videoSource) :
        SignallingClient(signallingServerConfiguration, webrtcConfiguration),
        m_videoSource(videoSource)
{

}

unique_ptr<PeerConnectionHandler> VideoStreamClient::createPeerConnectionHandler(const string& id,
        const Client& peerClient, bool isCaller)
{
    rtc::scoped_refptr<webrtc::VideoTrackInterface> videoTrack = m_peerConnectionFactory->CreateVideoTrack("stream", m_videoSource);

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
