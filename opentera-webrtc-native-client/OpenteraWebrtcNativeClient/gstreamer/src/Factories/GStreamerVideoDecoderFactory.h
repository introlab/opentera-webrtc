#pragma once

#include <gst/gst.h>
#include <api/video_codecs/video_decoder_factory.h>
#include "utils/gstreamer_helpers.h"

namespace opentera
{
    class GStreamerVideoDecoderFactory : public webrtc::VideoDecoderFactory
    {
    public:
        GStreamerVideoDecoderFactory();

    private:
        std::unique_ptr<webrtc::VideoDecoder> CreateVideoDecoder(const webrtc::SdpVideoFormat& format) final;
        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const final;
    };
}
