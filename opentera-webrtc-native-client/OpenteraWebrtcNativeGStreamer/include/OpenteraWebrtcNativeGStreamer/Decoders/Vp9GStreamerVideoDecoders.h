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

#include <OpenteraWebrtcNativeGStreamer/Decoders/GStreamerVideoDecoder.h>

namespace opentera
{
    class Vp9GStreamerVideoDecoder : public GStreamerVideoDecoder
    {
    public:
        explicit Vp9GStreamerVideoDecoder(std::string decoderPipeline, bool resetPipelineOnSizeChanges = false);
        ~Vp9GStreamerVideoDecoder() override = default;

        static const char* mediaTypeCaps();
        static const char* codecName();
    };

    class SoftwareVp9GStreamerVideoDecoder : public Vp9GStreamerVideoDecoder
    {
    public:
        SoftwareVp9GStreamerVideoDecoder();
        ~SoftwareVp9GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class VaapiVp9GStreamerVideoDecoder : public Vp9GStreamerVideoDecoder
    {
    public:
        VaapiVp9GStreamerVideoDecoder();
        ~VaapiVp9GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };

    class TegraVp9GStreamerVideoDecoder : public Vp9GStreamerVideoDecoder
    {
    public:
        TegraVp9GStreamerVideoDecoder();
        ~TegraVp9GStreamerVideoDecoder() override = default;

        [[nodiscard]] webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
        static bool isHardwareAccelerated();
    };
}

#endif
