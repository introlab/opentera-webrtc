#include <OpenteraWebrtcNativeClient/VideoStreamClient.h>
#include <OpenteraWebrtcNativeClient/Handlers/VideoStreamPeerConnectionHandler.h>

using namespace introlab;
using namespace std;

VideoStreamClient::VideoStreamClient(const SignallingServerConfiguration& signallingServerConfiguration,
        const WebrtcConfiguration& webrtcConfiguration) :
        SignallingClient(signallingServerConfiguration, webrtcConfiguration)
{

}

std::unique_ptr<PeerConnectionHandler> VideoStreamClient::createPeerConnectionHandler(const std::string& id,
        const Client& peerClient, bool isCaller)
{
    //m_peerConnectionFactory->CreateVideoTrack()
    
    return make_unique<VideoStreamPeerConnectionHandler>(id, peerClient, isCaller, getSendEventFunction(),
            getOnErrorFunction(), getOnClientConnectedFunction(), getOnClientDisconnectedFunction());
}
