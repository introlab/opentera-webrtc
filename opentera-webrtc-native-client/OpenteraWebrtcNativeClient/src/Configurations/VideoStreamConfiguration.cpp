#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>

using namespace opentera;
using namespace std;

VideoStreamConfiguration::VideoStreamConfiguration(
    unordered_set<VideoStreamCodec> forcedCodecs,
    bool forceGStreamerHardwareAcceleration,
    bool useGStreamerSoftwareEncoderDecoder)
    : m_forcedCodecs(move(forcedCodecs)),
      m_forceGStreamerHardwareAcceleration(forceGStreamerHardwareAcceleration),
      m_useGStreamerSoftwareEncoderDecoder(useGStreamerSoftwareEncoderDecoder)
{
}
