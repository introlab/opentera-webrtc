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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_ENCODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_WEBRTC_GSTREAMER_VIDEO_ENCODER_FACTORY_H

#include <api/video_codecs/sdp_video_format.h>
#include <api/video_codecs/video_encoder.h>
#include <api/video_codecs/video_encoder_factory.h>

#include <memory>
#include <vector>
#include <unordered_map>
#include <functional>

namespace opentera
{
    class WebRtcGStreamerVideoEncoderFactory : public webrtc::VideoEncoderFactory
    {
        struct EncoderFactory
        {
            int priority;
            bool isHardwareAccelerated;
            std::function<bool(const webrtc::CodecParameterMap&)> areParametersSupported;
            std::function<
                std::unique_ptr<webrtc::VideoEncoder>(const webrtc::Environment& env, const webrtc::SdpVideoFormat&)>
                factory;
        };

        std::unique_ptr<VideoEncoderFactory> m_builtinVideoEncoderFactory;
        std::vector<webrtc::SdpVideoFormat> m_builtinSupportedFormats;
        std::unordered_map<std::string, EncoderFactory> m_encoderFactories;

    public:
        WebRtcGStreamerVideoEncoderFactory(bool forceHardwareAcceleration, bool useGStreamerSoftwareEncoder);

        std::vector<webrtc::SdpVideoFormat> GetSupportedFormats() const override;
        CodecSupport QueryCodecSupport(
            const webrtc::SdpVideoFormat& format,
            absl::optional<std::string> scalabilityMode) const override;
        std::unique_ptr<webrtc::VideoEncoder>
            Create(const webrtc::Environment& env, const webrtc::SdpVideoFormat& format) override;

    private:
        void addH264Encoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareEncoder);
        void addVp8Encoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareEncoder);
        void addVp9Encoders(bool forceHardwareAcceleration, bool useGStreamerSoftwareEncoder);

        template<class Encoder>
        EncoderFactory createEncoderFactory(int priority);

        bool builtinVideoEncoderFactorySupports(std::string_view codecName);
        EncoderFactory createBuiltinEncoderFactory(int priority);
    };

    template<class Encoder>
    WebRtcGStreamerVideoEncoderFactory::EncoderFactory
        WebRtcGStreamerVideoEncoderFactory::createEncoderFactory(int priority)
    {
        return {
            priority,
            Encoder::isHardwareAccelerated(),
            [](auto parameters) { return Encoder::areParametersSupported(parameters); },
            []([[maybe_unused]] auto env, auto format) { return std::make_unique<Encoder>(format.parameters); }};
    }
}

#endif
