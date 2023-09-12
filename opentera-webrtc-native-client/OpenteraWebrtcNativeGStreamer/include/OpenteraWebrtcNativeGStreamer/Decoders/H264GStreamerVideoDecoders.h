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

#include <OpenteraWebrtcNativeGStreamer/Decoders/GStreamerVideoDecoder.h>

namespace opentera
{
    class H264GStreamerVideoDecoder : public GStreamerVideoDecoder
    {
    public:
        explicit H264GStreamerVideoDecoder(std::string decoderPipeline, bool resetPipelineOnSizeChanges = false);
        ~H264GStreamerVideoDecoder() override = default;

        static const char* mediaTypeCaps();
        static const char* codecName();
    };

    class SoftwareH264GStreamerVideoDecoder : public H264GStreamerVideoDecoder
    {
    public:
        SoftwareH264GStreamerVideoDecoder();
        ~SoftwareH264GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class VaapiH264GStreamerVideoDecoder : public H264GStreamerVideoDecoder
    {
    public:
        VaapiH264GStreamerVideoDecoder();
        ~VaapiH264GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class TegraH264GStreamerVideoDecoder : public H264GStreamerVideoDecoder
    {
    public:
        TegraH264GStreamerVideoDecoder();
        ~TegraH264GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class V4l2H264GStreamerVideoDecoder : public H264GStreamerVideoDecoder
    {
    public:
        V4l2H264GStreamerVideoDecoder();
        ~V4l2H264GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class AppleMediaH264GStreamerVideoDecoder : public H264GStreamerVideoDecoder
    {
    public:
        AppleMediaH264GStreamerVideoDecoder();
        ~AppleMediaH264GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };
}

#endif
