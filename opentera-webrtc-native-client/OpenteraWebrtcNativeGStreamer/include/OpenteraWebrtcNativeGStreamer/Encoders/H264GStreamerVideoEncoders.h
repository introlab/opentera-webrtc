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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_H264_VIDEO_DECODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_H264_VIDEO_DECODER_FACTORY_H

#include <OpenteraWebrtcNativeGStreamer/Encoders/GStreamerVideoEncoder.h>

namespace opentera
{
    class H264GStreamerVideoEncoder : public GStreamerVideoEncoder
    {
        webrtc::H264PacketizationMode m_packetizationMode;

    public:
        H264GStreamerVideoEncoder(
            const webrtc::CodecParameterMap& parameters,
            std::string encoderPipeline,
            std::string encoderBitratePropertyName,
            BitRateUnit bitRatePropertyUnit,
            std::string keyframeIntervalPropertyName,
            const char* additionalMediaTypeCaps = "");
        ~H264GStreamerVideoEncoder() override = default;

        static std::string mediaTypeCaps(const webrtc::CodecParameterMap& parameters);
        static const char* codecName();

        static bool isProfileBaselineOrMain(const webrtc::CodecParameterMap& parameters);
        static bool isProfileBaselineOrMainOrHigh444(const webrtc::CodecParameterMap& parameters);

    protected:
        int getKeyframeInterval(const webrtc::VideoCodec& codecSettings) override;

        void populateCodecSpecificInfo(
            webrtc::CodecSpecificInfo& codecSpecificInfo,
            const webrtc::EncodedImage& encodedFrame) override;
    };

    class SoftwareH264GStreamerVideoEncoder : public H264GStreamerVideoEncoder
    {
    public:
        explicit SoftwareH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~SoftwareH264GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class VaapiH264GStreamerVideoEncoder : public H264GStreamerVideoEncoder
    {
    public:
        explicit VaapiH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~VaapiH264GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class TegraH264GStreamerVideoEncoder : public H264GStreamerVideoEncoder
    {
    public:
        explicit TegraH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~TegraH264GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);

    private:
        static std::string profileFromParameters(const webrtc::CodecParameterMap& parameters);
    };

    class V4l2H264GStreamerVideoEncoder : public H264GStreamerVideoEncoder
    {
    public:
        explicit V4l2H264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~V4l2H264GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class AppleMediaH264GStreamerVideoEncoder : public H264GStreamerVideoEncoder
    {
    public:
        explicit AppleMediaH264GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~AppleMediaH264GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };
}

#endif
