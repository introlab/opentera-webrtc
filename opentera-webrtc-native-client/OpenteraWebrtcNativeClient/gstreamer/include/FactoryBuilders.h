#pragma once
#include <api/video_codecs/video_decoder_factory.h>
#include <api/video_codecs/video_encoder_factory.h>
#include <exception>

namespace opentera
{
    class BuiltWithoutGStreamerSupport : public std::runtime_error
    {
    public:
        BuiltWithoutGStreamerSupport() : std::runtime_error("Built without GStreamer support") {}
    };

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactory();
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactory();

    std::unique_ptr<webrtc::VideoDecoderFactory> createGStreamerVideoDecoderFactoryOrFallbackToBuiltin();
    std::unique_ptr<webrtc::VideoEncoderFactory> createGStreamerVideoEncoderFactoryOrFallbackToBuiltin();
}
