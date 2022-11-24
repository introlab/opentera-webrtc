#include "FactoryBuilders.h"

#include <api/video_codecs/builtin_video_decoder_factory.h>
#include <api/video_codecs/builtin_video_encoder_factory.h>
#include "Factories/GStreamerVideoDecoderFactory.h"

namespace opentera
{
#ifdef USE_GSTREAMER
    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactory()
    {
        return std::make_unique<opentera::GStreamerVideoDecoderFactory>();  // TODO
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactory()
    {
        return {};  // TODO
    }

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactoryOrFallbackToBuiltin()
    {
        auto factory = createGStreamerVideoDecoderFactory();
        if (factory)
        {
            return factory;
        }
        else
        {
            return webrtc::CreateBuiltinVideoDecoderFactory();
        }
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactoryOrFallbackToBuiltin()
    {
        auto factory = createGStreamerVideoEncoderFactory();
        if (factory)
        {
            return factory;
        }
        else
        {
            return webrtc::CreateBuiltinVideoEncoderFactory();
        }
    }
#else
    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactory()
    {
        throw BuiltWithoutGStreamerSupport();
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactory()
    {
        throw BuiltWithoutGStreamerSupport();
    }

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerDecoderVideoFactoryOrFallbackToBuiltin()
    {
        return webrtc::CreateBuiltinVideoDecoderFactory();
    }
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerEncoderVideoFactoryOrFallbackToBuiltin()
    {
        return webrtc::CreateBuiltinVideoDecoderFactory();
    }
#endif
}