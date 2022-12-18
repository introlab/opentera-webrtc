#include <OpenteraWebrtcNativeGStreamer/Factories/WebRtcGStreamerVideoDecoderFactory.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/H264GStreamerVideoDecoder.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp8GStreamerVideoDecoder.h>
#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp9GStreamerVideoDecoder.h>

#include <media/base/codec.h>

#include <algorithm>

using namespace opentera;
using namespace std;

WebRtcGStreamerVideoDecoderFactory::WebRtcGStreamerVideoDecoderFactory(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder) :
      m_builtinVideoDecoderFactory(webrtc::CreateBuiltinVideoDecoderFactory())
{
    m_builtinSupportedFormats = m_builtinVideoDecoderFactory->GetSupportedFormats();

    addH264Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
    addVp8Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
    addVp9Decoders(forceHardwareAcceleration, useGStreamerSoftwareDecoder);
}

vector<webrtc::SdpVideoFormat> WebRtcGStreamerVideoDecoderFactory::GetSupportedFormats() const
{
    vector<webrtc::SdpVideoFormat> supportedFormats;
    for(auto& kv : m_decoderFactories)
    {
        supportedFormats.emplace_back(kv.first);
    }
    sort(supportedFormats.begin(), supportedFormats.end(), [this](const webrtc::SdpVideoFormat& a, const webrtc::SdpVideoFormat& b)
    {
         return m_decoderFactories.at(a.name).priority < m_decoderFactories.at(b.name).priority;
    });
    return supportedFormats;
}

unique_ptr<webrtc::VideoDecoder> WebRtcGStreamerVideoDecoderFactory::CreateVideoDecoder(const webrtc::SdpVideoFormat& format)
{
    auto it = m_decoderFactories.find(format.name);
    if (it == m_decoderFactories.end())
    {
        return nullptr;
    }
    else
    {
        return it->second.factory(format);
    }
}

void WebRtcGStreamerVideoDecoderFactory::addH264Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder)
{
    auto H264_CODEC_NAME = H264GStreamerVideoDecoder::codecName();

    if (VaapiH264GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[H264_CODEC_NAME] = createDecoderFactory<VaapiH264GStreamerVideoDecoder>(1);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareH264GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[H264_CODEC_NAME] = createDecoderFactory<SoftwareH264GStreamerVideoDecoder>(2);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(H264_CODEC_NAME))
        {
            m_decoderFactories[H264_CODEC_NAME] = createBuiltinDecoderFactory(2);
        }
    }
}

void WebRtcGStreamerVideoDecoderFactory::addVp8Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder)
{
    auto VP8_CODEC_NAME = Vp8GStreamerVideoDecoder::codecName();

    if (VaapiVp8GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP8_CODEC_NAME] = createDecoderFactory<VaapiVp8GStreamerVideoDecoder>(1);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareVp8GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[VP8_CODEC_NAME] = createDecoderFactory<SoftwareVp8GStreamerVideoDecoder>(2);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(VP8_CODEC_NAME))
        {
            m_decoderFactories[VP8_CODEC_NAME] = createBuiltinDecoderFactory(2);
        }
    }
}

void WebRtcGStreamerVideoDecoderFactory::addVp9Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder)
{
    auto VP9_CODEC_NAME = Vp9GStreamerVideoDecoder::codecName();

    if (VaapiVp9GStreamerVideoDecoder::isSupported())
    {
        m_decoderFactories[VP9_CODEC_NAME] = createDecoderFactory<VaapiVp9GStreamerVideoDecoder>(1);
    }
    else if (!forceHardwareAcceleration)
    {
        if (useGStreamerSoftwareDecoder && SoftwareVp9GStreamerVideoDecoder::isSupported())
        {
            m_decoderFactories[VP9_CODEC_NAME] = createDecoderFactory<SoftwareVp9GStreamerVideoDecoder>(3);
        }
        else if (!useGStreamerSoftwareDecoder && builtinVideoDecoderFactorySupports(VP9_CODEC_NAME))
        {
            m_decoderFactories[VP9_CODEC_NAME] = createBuiltinDecoderFactory(2);
        }
    }
}

bool WebRtcGStreamerVideoDecoderFactory::builtinVideoDecoderFactorySupports(string_view codecName)
{
    return any_of(m_builtinSupportedFormats.begin(), m_builtinSupportedFormats.end(), [codecName](const webrtc::SdpVideoFormat& format)
           {
                return format.name == codecName;
           });
}

WebRtcGStreamerVideoDecoderFactory::DecoderFactory WebRtcGStreamerVideoDecoderFactory::createBuiltinDecoderFactory(int priority)
{
    return {
        priority,
            [this](const webrtc::SdpVideoFormat& format){return m_builtinVideoDecoderFactory->CreateVideoDecoder(format);}
    };
}
