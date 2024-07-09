#include <OpenteraWebrtcNativeClient/Codecs/VideoCodecFactories.h>

#ifdef USE_GSTREAMER
#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoEncoderFactory.h>
#else
#include <api/video_codecs/video_decoder_factory_template.h>
#include <api/video_codecs/video_decoder_factory_template_libvpx_vp8_adapter.h>
#include <api/video_codecs/video_decoder_factory_template_libvpx_vp9_adapter.h>
#include <api/video_codecs/video_decoder_factory_template_open_h264_adapter.h>

#include <api/video_codecs/video_encoder_factory_template.h>
#include <api/video_codecs/video_encoder_factory_template_libvpx_vp8_adapter.h>
#include <api/video_codecs/video_encoder_factory_template_libvpx_vp9_adapter.h>
#include <api/video_codecs/video_encoder_factory_template_open_h264_adapter.h>
#endif

#include <api/video_codecs/video_decoder.h>
#include <api/video_codecs/video_encoder.h>
#include <media/base/media_constants.h>

#include <unordered_map>
#include <algorithm>

using namespace opentera;
using namespace std;

static const unordered_map<string, VideoStreamCodec> CodecNameToVideoStreamCodec{
    {cricket::kVp8CodecName, VideoStreamCodec::VP8},
    {cricket::kVp9CodecName, VideoStreamCodec::VP9},
    {cricket::kH264CodecName, VideoStreamCodec::H264}};

static vector<webrtc::SdpVideoFormat> filterSupportedFormats(
    const vector<webrtc::SdpVideoFormat>& supportedFormats,
    const unordered_set<VideoStreamCodec>& forcedCodecs)
{
    if (forcedCodecs.empty())
    {
        return supportedFormats;
    }

    vector<webrtc::SdpVideoFormat> forcedSupportedFormats;
    copy_if(
        supportedFormats.begin(),
        supportedFormats.end(),
        back_inserter(forcedSupportedFormats),
        [&forcedCodecs](const webrtc::SdpVideoFormat& format)
        {
            auto it = CodecNameToVideoStreamCodec.find(format.name);
            if (it == CodecNameToVideoStreamCodec.end())
            {
                return false;
            }
            else
            {
                return forcedCodecs.count(it->second) > 0;
            }
        });

    return forcedSupportedFormats;
}

template<class Factory, class T>
static typename Factory::CodecSupport queryCodecSupport(
    const unique_ptr<Factory>& factory,
    const webrtc::SdpVideoFormat& format,
    T parameter,
    const unordered_set<VideoStreamCodec>& forcedCodecs)
{
    if (forcedCodecs.empty())
    {
        return factory->QueryCodecSupport(format, parameter);
    }

    typename Factory::CodecSupport codecSupport;
    auto it = CodecNameToVideoStreamCodec.find(format.name);
    if (it != CodecNameToVideoStreamCodec.end() && forcedCodecs.count(it->second) > 0)
    {
        codecSupport = factory->QueryCodecSupport(format, parameter);
    }
    else
    {
        codecSupport.is_supported = false;
        codecSupport.is_power_efficient = false;
    }

    return codecSupport;
}

ForcedCodecVideoDecoderFactory::ForcedCodecVideoDecoderFactory(
    unique_ptr<webrtc::VideoDecoderFactory> factory,
    unordered_set<VideoStreamCodec> forcedCodecs)
    : m_factory(move(factory)),
      m_forcedCodecs(move(forcedCodecs))
{
}

vector<webrtc::SdpVideoFormat> ForcedCodecVideoDecoderFactory::GetSupportedFormats() const
{
    return filterSupportedFormats(m_factory->GetSupportedFormats(), m_forcedCodecs);
}

webrtc::VideoDecoderFactory::CodecSupport
    ForcedCodecVideoDecoderFactory::QueryCodecSupport(const webrtc::SdpVideoFormat& format, bool referenceScaling) const
{
    return queryCodecSupport(m_factory, format, referenceScaling, m_forcedCodecs);
}

unique_ptr<webrtc::VideoDecoder>
    ForcedCodecVideoDecoderFactory::Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format)
{
    return m_factory->Create(env, format);
}


ForcedCodecVideoEncoderFactory::ForcedCodecVideoEncoderFactory(
    unique_ptr<webrtc::VideoEncoderFactory> factory,
    unordered_set<VideoStreamCodec> forcedCodecs)
    : m_factory(move(factory)),
      m_forcedCodecs(move(forcedCodecs))
{
}

vector<webrtc::SdpVideoFormat> ForcedCodecVideoEncoderFactory::GetSupportedFormats() const
{
    return filterSupportedFormats(m_factory->GetSupportedFormats(), m_forcedCodecs);
}

webrtc::VideoEncoderFactory::CodecSupport ForcedCodecVideoEncoderFactory::QueryCodecSupport(
    const webrtc::SdpVideoFormat& format,
    absl::optional<string> scalabilityMode) const
{
    return queryCodecSupport(m_factory, format, scalabilityMode, m_forcedCodecs);
}

unique_ptr<webrtc::VideoEncoder>
    ForcedCodecVideoEncoderFactory::Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format)
{
    return m_factory->Create(env, format);
}


#ifdef USE_GSTREAMER

unique_ptr<webrtc::VideoDecoderFactory>
    opentera::createVideoDecoderFactory(const VideoStreamConfiguration& configuration)
{
    auto gstreamerVideoDecoderFactory = make_unique<WebRtcGStreamerVideoDecoderFactory>(
        configuration.forceGStreamerHardwareAcceleration(),
        configuration.useGStreamerSoftwareEncoderDecoder());

    return make_unique<ForcedCodecVideoDecoderFactory>(
        move(gstreamerVideoDecoderFactory),
        configuration.forcedCodecs());
}

unique_ptr<webrtc::VideoEncoderFactory>
    opentera::createVideoEncoderFactory(const VideoStreamConfiguration& configuration)
{
    auto gstreamerVideoEncoderFactory = make_unique<WebRtcGStreamerVideoEncoderFactory>(
        configuration.forceGStreamerHardwareAcceleration(),
        configuration.useGStreamerSoftwareEncoderDecoder());

    return make_unique<ForcedCodecVideoEncoderFactory>(
        move(gstreamerVideoEncoderFactory),
        configuration.forcedCodecs());
}

#else

using BuiltinVideoDecoderFactory = webrtc::VideoDecoderFactoryTemplate<
    webrtc::LibvpxVp8DecoderTemplateAdapter,
    webrtc::LibvpxVp9DecoderTemplateAdapter,
    webrtc::OpenH264DecoderTemplateAdapter>;

unique_ptr<webrtc::VideoDecoderFactory>
    opentera::createVideoDecoderFactory(const VideoStreamConfiguration& configuration)
{
    return make_unique<ForcedCodecVideoDecoderFactory>(
        make_unique<BuiltinVideoDecoderFactory>(),
        configuration.forcedCodecs());
}

using BuiltinVideoEncoderFactory = webrtc::VideoEncoderFactoryTemplate<
    webrtc::LibvpxVp8EncoderTemplateAdapter,
    webrtc::LibvpxVp9EncoderTemplateAdapter,
    webrtc::OpenH264EncoderTemplateAdapter>;

unique_ptr<webrtc::VideoEncoderFactory>
    opentera::createVideoEncoderFactory(const VideoStreamConfiguration& configuration)
{
    return make_unique<ForcedCodecVideoEncoderFactory>(
        make_unique<BuiltinVideoEncoderFactory>(),
        configuration.forcedCodecs());
}

#endif
