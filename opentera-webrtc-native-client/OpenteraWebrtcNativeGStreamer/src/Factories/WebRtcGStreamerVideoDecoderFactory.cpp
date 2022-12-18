#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/H264GStreamerVideoDecoder.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp8GStreamerVideoDecoder.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp9GStreamerVideoDecoder.h>

#include <media/base/codec.h>

using namespace opentera;
using namespace std;

// TODO check supported codec
vector<webrtc::SdpVideoFormat> WebRtcGStreamerVideoDecoderFactory::GetSupportedFormats() const
{
    // Check if we need to set other parameters
    return {webrtc::SdpVideoFormat(SoftwareVp9GStreamerVideoDecoder::codecName())};
}

unique_ptr<webrtc::VideoDecoder> WebRtcGStreamerVideoDecoderFactory::CreateVideoDecoder(const webrtc::SdpVideoFormat& format)
{
    return make_unique<SoftwareVp9GStreamerVideoDecoder>();
}
