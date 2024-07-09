#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CODECS_VIDEO_CODEC_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CODECS_VIDEO_CODEC_H

#include <OpenteraWebrtcNativeClient/Configurations/VideoStreamConfiguration.h>
#include <OpenteraWebrtcNativeClient/Utils/ClassMacro.h>

#include <api/video_codecs/video_decoder_factory.h>
#include <api/video_codecs/video_encoder_factory.h>

namespace opentera
{
    class ForcedCodecVideoDecoderFactory : public webrtc::VideoDecoderFactory
    {
        std::unique_ptr<webrtc::VideoDecoderFactory> m_factory;
        std::unordered_set<VideoStreamCodec> m_forcedCodecs;

    public:
        ForcedCodecVideoDecoderFactory(
            std::unique_ptr<webrtc::VideoDecoderFactory> factory,
            std::unordered_set<VideoStreamCodec> forcedCodecs);
        ~ForcedCodecVideoDecoderFactory() override = default;

        DECLARE_NOT_COPYABLE(ForcedCodecVideoDecoderFactory);
        DECLARE_NOT_MOVABLE(ForcedCodecVideoDecoderFactory);

        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
        CodecSupport QueryCodecSupport(const webrtc::SdpVideoFormat& format, bool referenceScaling) const override;
        std::unique_ptr<webrtc::VideoDecoder> Create(
              const webrtc::Environment& env,
              const webrtc::SdpVideoFormat& format) override;
    };

    class ForcedCodecVideoEncoderFactory : public webrtc::VideoEncoderFactory
    {
        std::unique_ptr<webrtc::VideoEncoderFactory> m_factory;
        std::unordered_set<VideoStreamCodec> m_forcedCodecs;

    public:
        ForcedCodecVideoEncoderFactory(
            std::unique_ptr<webrtc::VideoEncoderFactory> factory,
            std::unordered_set<VideoStreamCodec> forcedCodecs);
        ~ForcedCodecVideoEncoderFactory() override = default;

        DECLARE_NOT_COPYABLE(ForcedCodecVideoEncoderFactory);
        DECLARE_NOT_MOVABLE(ForcedCodecVideoEncoderFactory);

        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;

        CodecSupport QueryCodecSupport(
            const webrtc::SdpVideoFormat& format,
            absl::optional<std::string> scalabilityMode) const override;

        // Creates a VideoEncoder for the specified format.
        std::unique_ptr<webrtc::VideoEncoder> Create(
            const webrtc::Environment& env,
            const webrtc::SdpVideoFormat& format) override;
    };

    std::unique_ptr<webrtc::VideoDecoderFactory>
        createVideoDecoderFactory(const VideoStreamConfiguration& configuration);
    std::unique_ptr<webrtc::VideoEncoderFactory>
        createVideoEncoderFactory(const VideoStreamConfiguration& configuration);
}

#endif
