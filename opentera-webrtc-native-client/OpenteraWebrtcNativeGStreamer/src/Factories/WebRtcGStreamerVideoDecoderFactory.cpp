#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Decoders/H264GStreamerVideoDecoders.h>
#include <OpenteraWebrtcNativeGStreamer/Decoders/Vp8GStreamerVideoDecoders.h>
#include <OpenteraWebrtcNativeGStreamer/Decoders/Vp9GStreamerVideoDecoders.h>

#include <media/base/codec.h>

#include <api/video_codecs/video_decoder_factory_template.h>
#include <api/video_codecs/video_decoder_factory_template_libvpx_vp8_adapter.h>
#include <api/video_codecs/video_decoder_factory_template_libvpx_vp9_adapter.h>
#include <api/video_codecs/video_decoder_factory_template_open_h264_adapter.h>

#include <algorithm>

using namespace opentera;
using namespace std;

constexpr int HardwarePriority = 1;
constexpr int SoftwarePriority = 2;

using BuiltinVideoDecoderFactory = webrtc::VideoDecoderFactoryTemplate<
    webrtc::LibvpxVp8DecoderTemplateAdapter,
    webrtc::LibvpxVp9DecoderTemplateAdapter,
    webrtc::OpenH264DecoderTemplateAdapter>;

WebRtcGStreamerVideoDecoderFactory::WebRtcGStreamerVideoDecoderFactory(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareDecoder)
    : m_builtinVideoDecoderFactory(make_unique<BuiltinVideoDecoderFactory>())
{
    m_builtinSupportedFormats = m_builtinVideoDecoderFactory->GetSupportedFormats();

    addH264Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
    addVp8Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
    addVp9Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
}

vector<webrtc::SdpVideoFormat> WebRtcGStreamerVideoDecoderFactory::GetSupportedFormats() const
{
    vector<webrtc::SdpVideoFormat> supportedFormats;
    supportedFormats.reserve(m_decoderFactories.size());
    for (auto& f : m_builtinSupportedFormats)
    {
        if (m_decoderFactories.find(f.name) != m_decoderFactories.end())
        {
            supportedFormats.emplace_back(f);
        }
    }
    sort(
        supportedFormats.begin(),
        supportedFormats.end(),
        [this](const webrtc::SdpVideoFormat& a, const webrtc::SdpVideoFormat& b)
        { return m_decoderFactories.at(a.name).priority < m_decoderFactories.at(b.name).priority; });
    return supportedFormats;
}

webrtc::VideoDecoderFactory::CodecSupport WebRtcGStreamerVideoDecoderFactory::QueryCodecSupport(
    const webrtc::SdpVideoFormat& format,
    bool referenceScaling) const
{
    CodecSupport codecSupport;

    auto it = m_decoderFactories.find(format.name);
    if (it == m_decoderFactories.end())
    {
        codecSupport.is_supported = false;
        codecSupport.is_power_efficient = false;
    }
    else
    {
        codecSupport.is_supported = !referenceScaling;
        codecSupport.is_power_efficient = it->second.isHardwareAccelerated;
    }

    return codecSupport;
}

unique_ptr<webrtc::VideoDecoder>
    WebRtcGStreamerVideoDecoderFactory::Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format)
{
    auto it = m_decoderFactories.find(format.name);
    if (it == m_decoderFactories.end())
    {
        return nullptr;
    }
    else
    {
        return it->second.factory(env, format);
    }
}

void WebRtcGStreamerVideoDecoderFactory::addH264Decoders(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareDecoder)
{
    auto H264_CODEC_NAME = H264GStreamerVideoDecoder::codecName();

    if (VaapiH264GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[H264_CODEC_NAME] = createDecoderFactory<VaapiH264GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (TegraH264GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[H264_CODEC_NAME] = createDecoderFactory<TegraH264GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (V4l2H264GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[H264_CODEC_NAME] = createDecoderFactory<V4l2H264GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (AppleMediaH264GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[H264_CODEC_NAME] =
            createDecoderFactory<AppleMediaH264GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareH264GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[H264_CODEC_NAME] =
                createDecoderFactory<SoftwareH264GStreamerVideoDecoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(H264_CODEC_NAME))
        {
            m_decoderFactories[H264_CODEC_NAME] = createBuiltinDecoderFactory(SoftwarePriority);
        }
    }
}

void WebRtcGStreamerVideoDecoderFactory::addVp8Decoders(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareDecoder)
{
    auto VP8_CODEC_NAME = Vp8GStreamerVideoDecoder::codecName();

    if (VaapiVp8GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP8_CODEC_NAME] = createDecoderFactory<VaapiVp8GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (TegraVp8GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP8_CODEC_NAME] = createDecoderFactory<TegraVp8GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareVp8GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[VP8_CODEC_NAME] =
                createDecoderFactory<SoftwareVp8GStreamerVideoDecoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(VP8_CODEC_NAME))
        {
            m_decoderFactories[VP8_CODEC_NAME] = createBuiltinDecoderFactory(SoftwarePriority);
        }
    }
}

void WebRtcGStreamerVideoDecoderFactory::addVp9Decoders(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareDecoder)
{
    auto VP9_CODEC_NAME = Vp9GStreamerVideoDecoder::codecName();

    if (VaapiVp9GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP9_CODEC_NAME] = createDecoderFactory<VaapiVp9GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (TegraVp9GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP9_CODEC_NAME] = createDecoderFactory<TegraVp9GStreamerVideoDecoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareVp9GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[VP9_CODEC_NAME] =
                createDecoderFactory<SoftwareVp9GStreamerVideoDecoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(VP9_CODEC_NAME))
        {
            m_decoderFactories[VP9_CODEC_NAME] = createBuiltinDecoderFactory(SoftwarePriority);
        }
    }
}

bool WebRtcGStreamerVideoDecoderFactory::builtinVideoDecoderFactorySupports(string_view codecName)
{
    return any_of(
        m_builtinSupportedFormats.begin(),
        m_builtinSupportedFormats.end(),
        [codecName](const webrtc::SdpVideoFormat& format) { return format.name == codecName; });
}

WebRtcGStreamerVideoDecoderFactory::DecoderFactory
    WebRtcGStreamerVideoDecoderFactory::createBuiltinDecoderFactory(int priority)
{
    return {priority, false, [this](const webrtc::Environment& env, const webrtc::SdpVideoFormat& format) {
                return m_builtinVideoDecoderFactory->Create(env, format);
            }};
}
