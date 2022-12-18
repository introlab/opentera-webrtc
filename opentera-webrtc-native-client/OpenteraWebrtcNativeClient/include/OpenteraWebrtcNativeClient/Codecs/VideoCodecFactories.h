#ifndef OPENTERA_WEBRTC_NATIVE_CLIENT_CODECS_VIDEO_CODEC_H
#define OPENTERA_WEBRTC_NATIVE_CLIENT_CODECS_VIDEO_CODEC_H

#include <api/video_codecs/video_decoder_factory.h>
#include <api/video_codecs/video_encoder_factory.h>

namespace opentera
{
    std::unique_ptr<webrtc::VideoDecoderFactory> createVideoDecoderFactory();
    std::unique_ptr<webrtc::VideoEncoderFactory> createVideoEncoderFactory();
}

#endif
