#include <OpenteraWebrtcNativeClient/Configurations/DataChannelConfiguration.h>

using namespace opentera;
using namespace std;

DataChannelConfiguration::DataChannelConfiguration(
    bool ordered,
    absl::optional<int> maxPacketLifeTime,
    absl::optional<int> maxRetransmits,
    string&& protocol)
    : m_ordered(ordered),
      m_maxPacketLifeTime(maxPacketLifeTime),
      m_maxRetransmits(maxRetransmits),
      m_protocol(move(protocol))
{
}

/**
 * Converts a AudioSourceConfiguration to a webrtc::DataChannelInit.
 * @return The converted webrtc::DataChannelInit
 */
DataChannelConfiguration::operator webrtc::DataChannelInit() const
{
    webrtc::DataChannelInit configuration;
    configuration.ordered = m_ordered;
    if (m_maxPacketLifeTime)
    {
        configuration.maxRetransmitTime = m_maxPacketLifeTime.value();
    }
    if (m_maxRetransmits)
    {
        configuration.maxRetransmits = m_maxRetransmits.value();
    }
    configuration.protocol = m_protocol;

    return configuration;
}
