#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>

using namespace opentera;
using namespace std;

absl::optional<VideoStreamCodec> opentera::stringToVideoStreamCodec(string_view value)
{
    if (value == "VP8" || value == "vp8" || value == "vP8" || value == "Vp8")
    {
        return VideoStreamCodec::VP8;
    }
    else if (value == "VP9" || value == "vp9" || value == "vP9" || value == "Vp9")
    {
        return VideoStreamCodec::VP9;
    }
    else if (value == "H264" || value == "h264")
    {
        return VideoStreamCodec::H264;
    }
    else
    {
        return absl::nullopt;
    }
}

VideoStreamConfiguration::VideoStreamConfiguration(
    unordered_set<VideoStreamCodec> forcedCodecs,
    bool forceGStreamerHardwareAcceleration,
    bool useGStreamerSoftwareEncoderDecoder)
    : m_forcedCodecs(move(forcedCodecs)),
      m_forceGStreamerHardwareAcceleration(forceGStreamerHardwareAcceleration),
      m_useGStreamerSoftwareEncoderDecoder(useGStreamerSoftwareEncoderDecoder)
{
}
