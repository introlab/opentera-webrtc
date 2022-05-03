#include <OpenteraWebrtcNativeClient/Configurations/WebrtcConfiguration.h>

using namespace opentera;
using namespace std;

WebrtcConfiguration::WebrtcConfiguration(vector<IceServer>&& iceServers) : m_iceServers(move(iceServers)) {}

/**
 * Converts a AudioSourceConfiguration to a
 * webrtc::PeerConnectionInterface::RTCConfiguration.
 * @return The converted webrtc::PeerConnectionInterface::RTCConfiguration
 */
WebrtcConfiguration::operator webrtc::PeerConnectionInterface::RTCConfiguration() const
{
    webrtc::PeerConnectionInterface::RTCConfiguration configuration;
    for (const auto& iceServer : m_iceServers)
    {
        configuration.servers.push_back(static_cast<webrtc::PeerConnectionInterface::IceServer>(iceServer));
    }

    configuration.sdp_semantics = webrtc::SdpSemantics::kUnifiedPlan;

    return configuration;
}
