/*
 *  Copyright (C) 2022 IntRoLab
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Library General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VP8_VIDEO_DECODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VP8_VIDEO_DECODER_FACTORY_H

#include <OpenteraWebrtcNativeGStreamer/Encoders/GStreamerVideoEncoder.h>

namespace opentera
{
    class Vp8GStreamerVideoEncoder : public GStreamerVideoEncoder
    {
    public:
        Vp8GStreamerVideoEncoder(
            std::string encoderPipeline,
            std::string encoderBitratePropertyName,
            BitRateUnit bitRatePropertyUnit,
            std::string keyframeIntervalPropertyName);
        ~Vp8GStreamerVideoEncoder() override = default;

        static const char* mediaTypeCaps();
        static const char* codecName();

    protected:
        int getKeyframeInterval(const webrtc::VideoCodec& codecSettings) override;

        void populateCodecSpecificInfo(
            webrtc::CodecSpecificInfo& codecSpecificInfo,
            const webrtc::EncodedImage& encodedFrame) override;
    };

    class SoftwareVp8GStreamerVideoEncoder : public Vp8GStreamerVideoEncoder
    {
    public:
        explicit SoftwareVp8GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~SoftwareVp8GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class VaapiVp8GStreamerVideoEncoder : public Vp8GStreamerVideoEncoder
    {
    public:
        explicit VaapiVp8GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~VaapiVp8GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class TegraVp8GStreamerVideoEncoder : public Vp8GStreamerVideoEncoder
    {
    public:
        explicit TegraVp8GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~TegraVp8GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };
}

#endif
