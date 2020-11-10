#include <OpenteraWebrtcNativeClient/VideoStreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;
using namespace rtc;
using namespace webrtc;

VideoStreamClient::VideoStreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
        const WebrtcConfiguration& webrtcConfiguration) :
        SignallingClient(signallingServerConfiguration, webrtcConfiguration)
{

}

unique_ptr<PeerConnectionHandler> VideoStreamClient::createPeerConnectionHandler(const string& id,
        const Client& peerClient, bool isCaller)
{
        VideoTrackSourceInterface* source;
        auto videoTrack = m_peerConnectionFactory->CreateVideoTrack("stream", source);

    return make_unique<VideoStreamPeerConnectionHandler>(id, peerClient, isCaller, getSendEventFunction(),
            getOnErrorFunction(), getOnClientConnectedFunction(), getOnClientDisconnectedFunction());
}
