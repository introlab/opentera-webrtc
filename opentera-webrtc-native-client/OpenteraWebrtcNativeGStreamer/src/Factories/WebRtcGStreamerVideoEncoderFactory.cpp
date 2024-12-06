#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoEncoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Encoders/H264GStreamerVideoEncoders.h>
#include <OpenteraWebrtcNativeGStreamer/Encoders/Vp8GStreamerVideoEncoders.h>
#include <OpenteraWebrtcNativeGStreamer/Encoders/Vp9GStreamerVideoEncoders.h>

#include <media/base/codec.h>

#include <api/video_codecs/video_encoder_factory_template.h>
#include <api/video_codecs/video_encoder_factory_template_libvpx_vp8_adapter.h>
#include <api/video_codecs/video_encoder_factory_template_libvpx_vp9_adapter.h>
#include <api/video_codecs/video_encoder_factory_template_open_h264_adapter.h>

#include <algorithm>

using namespace opentera;
using namespace std;

constexpr int HardwarePriority = 1;
constexpr int SoftwarePriority = 2;

using BuiltinVideoEncoderFactory = webrtc::VideoEncoderFactoryTemplate<
    webrtc::LibvpxVp8EncoderTemplateAdapter,
    webrtc::LibvpxVp9EncoderTemplateAdapter,
    webrtc::OpenH264EncoderTemplateAdapter>;

WebRtcGStreamerVideoEncoderFactory::WebRtcGStreamerVideoEncoderFactory(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareEncoder)
    : m_builtinVideoEncoderFactory(make_unique<BuiltinVideoEncoderFactory>())
{
    m_builtinSupportedFormats = m_builtinVideoEncoderFactory->GetSupportedFormats();

    addH264Encoders(forceHardwareAcceleration, useGStreamerSoftwareEncoder);
    addVp8Encoders(forceHardwareAcceleration, useGStreamerSoftwareEncoder);
    addVp9Encoders(forceHardwareAcceleration, useGStreamerSoftwareEncoder);
}

vector<webrtc::SdpVideoFormat> WebRtcGStreamerVideoEncoderFactory::GetSupportedFormats() const
{
    vector<webrtc::SdpVideoFormat> supportedFormats;
    supportedFormats.reserve(m_encoderFactories.size());
    for (auto& f : m_builtinSupportedFormats)
    {
        auto it = m_encoderFactories.find(f.name);
        if (it != m_encoderFactories.end() && it->second.areParametersSupported(f.parameters))
        {
            supportedFormats.emplace_back(f);
        }
    }
    sort(
        supportedFormats.begin(),
        supportedFormats.end(),
        [this](const webrtc::SdpVideoFormat& a, const webrtc::SdpVideoFormat& b)
        { return m_encoderFactories.at(a.name).priority < m_encoderFactories.at(b.name).priority; });
    return supportedFormats;
}

webrtc::VideoEncoderFactory::CodecSupport WebRtcGStreamerVideoEncoderFactory::QueryCodecSupport(
    const webrtc::SdpVideoFormat& format,
    absl::optional<string> scalabilityMode) const
{
    CodecSupport codecSupport;

    auto it = m_encoderFactories.find(format.name);
    if (it == m_encoderFactories.end())
    {
        codecSupport.is_supported = false;
        codecSupport.is_power_efficient = false;
    }
    else
    {
        codecSupport.is_supported = !scalabilityMode.has_value();
        codecSupport.is_power_efficient = it->second.isHardwareAccelerated;
    }

    return codecSupport;
}

unique_ptr<webrtc::VideoEncoder>
    WebRtcGStreamerVideoEncoderFactory::Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format)
{
    auto it = m_encoderFactories.find(format.name);
    if (it == m_encoderFactories.end())
    {
        return nullptr;
    }
    else
    {
        return it->second.factory(env, format);
    }
}

void WebRtcGStreamerVideoEncoderFactory::addH264Encoders(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareEncoder)
{
    auto H264_CODEC_NAME = H264GStreamerVideoEncoder::codecName();

    if (VaapiH264GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[H264_CODEC_NAME] = createEncoderFactory<VaapiH264GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (TegraH264GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[H264_CODEC_NAME] = createEncoderFactory<TegraH264GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (V4l2H264GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[H264_CODEC_NAME] = createEncoderFactory<V4l2H264GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (AppleMediaH264GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[H264_CODEC_NAME] =
            createEncoderFactory<AppleMediaH264GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareEncoder && SoftwareH264GStreamerVideoEncoder::isSupported())
        {
            m_encoderFactories[H264_CODEC_NAME] =
                createEncoderFactory<SoftwareH264GStreamerVideoEncoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareEncoder && builtinVideoEncoderFactorySupports(H264_CODEC_NAME))
        {
            m_encoderFactories[H264_CODEC_NAME] = createBuiltinEncoderFactory(SoftwarePriority);
        }
    }
}

void WebRtcGStreamerVideoEncoderFactory::addVp8Encoders(
    bool forceHardwareAcceleration,
    bool useGStreamerSoftwareEncoder)
{
    auto VP8_CODEC_NAME = Vp8GStreamerVideoEncoder::codecName();

    if (VaapiVp8GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[VP8_CODEC_NAME] = createEncoderFactory<VaapiVp8GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (TegraVp8GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[VP8_CODEC_NAME] = createEncoderFactory<TegraVp8GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareEncoder && SoftwareVp8GStreamerVideoEncoder::isSupported())
        {
            m_encoderFactories[VP8_CODEC_NAME] =
                createEncoderFactory<SoftwareVp8GStreamerVideoEncoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareEncoder && builtinVideoEncoderFactorySupports(VP8_CODEC_NAME))
        {
            m_encoderFactories[VP8_CODEC_NAME] = createBuiltinEncoderFactory(SoftwarePriority);
        }
    }
}

void WebRtcGStreamerVideoEncoderFactory::addVp9Encoders(
    // TODO Remove maybe_unused when VP9 works.
    [[maybe_unused]] bool forceHardwareAcceleration,
    bool useGStreamerSoftwareEncoder)
{
    auto VP9_CODEC_NAME = Vp9GStreamerVideoEncoder::codecName();

    // TODO Remove comments when VP9 works.
    /*if (TegraVp9GStreamerVideoEncoder::isSupported())
    {
        m_encoderFactories[VP9_CODEC_NAME] = createEncoderFactory<TegraVp9GStreamerVideoEncoder>(HardwarePriority);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareEncoder && SoftwareVp9GStreamerVideoEncoder::isSupported())
        {
            m_encoderFactories[VP9_CODEC_NAME] =
                createEncoderFactory<SoftwareVp9GStreamerVideoEncoder>(SoftwarePriority);
        }
        else if (!useGStreamerSoftwareEncoder && builtinVideoEncoderFactorySupports(VP9_CODEC_NAME))
        {
            m_encoderFactories[VP9_CODEC_NAME] = createBuiltinEncoderFactory(SoftwarePriority);
        }
    }*/

    // TODO Remove when VP9 works.
    if (!useGStreamerSoftwareEncoder && builtinVideoEncoderFactorySupports(VP9_CODEC_NAME))
    {
        m_encoderFactories[VP9_CODEC_NAME] = createBuiltinEncoderFactory(SoftwarePriority);
    }
}

bool WebRtcGStreamerVideoEncoderFactory::builtinVideoEncoderFactorySupports(string_view codecName)
{
    return any_of(
        m_builtinSupportedFormats.begin(),
        m_builtinSupportedFormats.end(),
        [codecName](const webrtc::SdpVideoFormat& format) { return format.name == codecName; });
}

WebRtcGStreamerVideoEncoderFactory::EncoderFactory
    WebRtcGStreamerVideoEncoderFactory::createBuiltinEncoderFactory(int priority)
{
    return {
        priority,
        false,
        []([[maybe_unused]] const webrtc::CodecParameterMap& parameters) { return true; },
        [this](const webrtc::Environment& env, const webrtc::SdpVideoFormat& format)
        { return m_builtinVideoEncoderFactory->Create(env, format); }};
}
