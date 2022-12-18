#include <OpenteraWebrtcNativeClient/Codecs/VideoCodecFactories.h>
#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>

#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>

using namespace opentera;
using namespace std;

#ifdef USE_GSTREAMER

unique_ptr<webrtc::VideoDecoderFactory> opentera::createVideoDecoderFactory()
{
    return make_unique<WebRtcGStreamerVideoDecoderFactory>();
}

unique_ptr<webrtc::VideoEncoderFactory> opentera::createVideoEncoderFactory()
{
    return webrtc::CreateBuiltinVideoEncoderFactory(); // TODO change to GStreamer
}

#else

unique_ptr<webrtc::VideoDecoderFactory> opentera::createVideoDecoderFactory()
{
    return webrtc::CreateBuiltinVideoDecoderFactory();
}

unique_ptr<webrtc::VideoEncoderFactory> opentera::createVideoEncoderFactory()
{
    return webrtc::CreateBuiltinVideoEncoderFactory();
}

#endif
