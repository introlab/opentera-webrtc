#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/GStreamerVideoDecoder.h>

#include <media/base/codec.h>

using namespace opentera;
using namespace std;

// TODO check supported codec
vector<webrtc::SdpVideoFormat> WebRtcGStreamerVideoDecoderFactory::GetSupportedFormats() const
{
    // Check if we need to set other parameters
    return {webrtc::SdpVideoFormat(GStreamerVideoDecoder::Name())};
}

unique_ptr<webrtc::VideoDecoder> WebRtcGStreamerVideoDecoderFactory::CreateVideoDecoder(const webrtc::SdpVideoFormat& format)
{
    return make_unique<GStreamerVideoDecoder>();
}
