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

#ifndef OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VP9_VIDEO_DECODER_FACTORY_H
#define OPENTERA_WEBRTC_NATIVE_GSTREAMER_FACTORIES_GSTREAMER_VP9_VIDEO_DECODER_FACTORY_H

#include <OpenteraWebrtcNativeGStreamer/Encoders/GStreamerVideoEncoder.h>

namespace opentera
{
    class Vp9GStreamerVideoEncoder : public GStreamerVideoEncoder
    {
        bool m_isFlexibleMode;

    public:
        Vp9GStreamerVideoEncoder(
            const webrtc::CodecParameterMap& parameters,
            std::string encoderPipeline,
            std::string encoderBitratePropertyName,
            BitRateUnit bitRatePropertyUnit,
            std::string keyframeIntervalPropertyName);
        ~Vp9GStreamerVideoEncoder() override = default;

        int InitEncode(const webrtc::VideoCodec* codecSettings, const Settings& settings) override;

        static std::string mediaTypeCaps(const webrtc::CodecParameterMap& parameters);
        static const char* codecName();

        static bool isProfile0123OrNone(const webrtc::CodecParameterMap& parameters);

    protected:
        int getKeyframeInterval(const webrtc::VideoCodec& codecSettings) override;

        void populateCodecSpecificInfo(
            webrtc::CodecSpecificInfo& codecSpecificInfo,
            const webrtc::EncodedImage& encodedFrame) override;
    };

    class SoftwareVp9GStreamerVideoEncoder : public Vp9GStreamerVideoEncoder
    {
    public:
        explicit SoftwareVp9GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~SoftwareVp9GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };

    class TegraVp9GStreamerVideoEncoder : public Vp9GStreamerVideoEncoder
    {
    public:
        explicit TegraVp9GStreamerVideoEncoder(const webrtc::CodecParameterMap& parameters);
        ~TegraVp9GStreamerVideoEncoder() override = default;

        [[nodiscard]] webrtc::VideoEncoder::EncoderInfo GetEncoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
        static bool areParametersSupported(const webrtc::CodecParameterMap& parameters);
    };
}

#endif
