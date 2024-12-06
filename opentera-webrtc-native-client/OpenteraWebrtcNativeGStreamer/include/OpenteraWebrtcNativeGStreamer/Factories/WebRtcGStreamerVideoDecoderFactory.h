/*
 *  Copyright 2022 IntRoLab
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_DECODER_FACOTRY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_DECODER_FACOTRY_H

#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_decoder.h>
#include <api/video_codecs/video_decoder_factory.h>

#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>

namespace opentera
{
    class WebRtcGStreamerVideoDecoderFactory : public webrtc::VideoDecoderFactory
    {
        struct DecoderFactory
        {
            int priority;
            bool isHardwareAccelerated;
            std::function<
                std::unique_ptr<webrtc::VideoDecoder>(const webrtc::Environment& env, const webrtc::SdpVideoFormat&)>
                factory;
        };

        std::unique_ptr<VideoDecoderFactory> m_builtinVideoDecoderFactory;
        std::vector<webrtc::SdpVideoFormat> m_builtinSupportedFormats;
        std::unordered_map<std::string, DecoderFactory> m_decoderFactories;

    public:
        WebRtcGStreamerVideoDecoderFactory(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder);

        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
        CodecSupport QueryCodecSupport(const webrtc::SdpVideoFormat& format, bool referenceScaling) const override;
        std::unique_ptr<webrtc::VideoDecoder>
            Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format) override;

    private:
        void addH264Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder);
        void addVp8Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder);
        void addVp9Decoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareDecoder);

        template<class Decoder>
        DecoderFactory createDecoderFactory(int priority);

        bool builtinVideoDecoderFactorySupports(std::string_view codecName);
        DecoderFactory createBuiltinDecoderFactory(int priority);
    };

    template<class Decoder>
    WebRtcGStreamerVideoDecoderFactory::DecoderFactory
        WebRtcGStreamerVideoDecoderFactory::createDecoderFactory(int priority)
    {
        return {
            priority,
            Decoder::isHardwareAccelerated(),
            []([[maybe_unused]] auto _env, [[maybe_unused]] auto _format) { return std::make_unique<Decoder>(); }};
    }
}

#endif
