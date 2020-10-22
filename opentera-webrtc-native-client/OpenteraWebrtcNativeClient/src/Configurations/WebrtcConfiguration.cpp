#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>

using namespace introlab;
using namespace std;

WebrtcConfiguration::WebrtcConfiguration(const vector<IceServer>& iceServers) : m_iceServers(iceServers)
{
}

WebrtcConfiguration::operator webrtc::PeerConnectionInterface::RTCConfiguration() const
{
    webrtc::PeerConnectionInterface::RTCConfiguration configuration;
    for (const auto& iceServer : m_iceServers)
    {
        configuration.servers.push_back(static_cast<webrtc::PeerConnectionInterface::IceServer>(iceServer));
    }

    return configuration;
}
