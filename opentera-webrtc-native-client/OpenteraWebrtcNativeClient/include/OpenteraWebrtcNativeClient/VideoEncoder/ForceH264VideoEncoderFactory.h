#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_ENCODER_FORCE_H264_VIDEO_ENCODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_VIDEO_ENCODER_FORCE_H264_VIDEO_ENCODER_FACTORY_H

#include <api/video_codecs/video_encoder_factory.h>
#include <api/video_codecs/video_encoder.h>

namespace opentera
{
    class ForceH264VideoEncoderFactory : public webrtc::VideoEncoderFactory
    {
        std::unique_ptr<webrtc::VideoEncoderFactory> m_videoEncoderFactory;

    public:
        explicit ForceH264VideoEncoderFactory(std::unique_ptr<webrtc::VideoEncoderFactory> videoEncoderFactory);
        ~ForceH264VideoEncoderFactory() override;

        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
        std::vector<webrtc::SdpVideoFormat> GetImplementations() const override;

        CodecInfo QueryVideoEncoder(const webrtc::SdpVideoFormat& format) const override;
        std::unique_ptr<webrtc::VideoEncoder> CreateVideoEncoder(const webrtc::SdpVideoFormat& format) override;

        std::unique_ptr<EncoderSelectorInterface> GetEncoderSelector() const override;

    private:
        static std::vector<webrtc::SdpVideoFormat> filterSdpFormats(const std::vector<webrtc::SdpVideoFormat>& formats);
    };

    std::unique_ptr<webrtc::VideoEncoderFactory>
        createForceH264VideoEncoderFactory(std::unique_ptr<webrtc::VideoEncoderFactory> videoEncoderFactory);
}

#endif
