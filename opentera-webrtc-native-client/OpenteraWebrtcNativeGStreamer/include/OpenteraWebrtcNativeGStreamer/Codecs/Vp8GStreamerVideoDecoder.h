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

#include <OpenteraWebrtcNativeGStreamer/Codecs/GStreamerVideoDecoder.h>

namespace opentera
{
    class Vp8GStreamerVideoDecoder : public GStreamerVideoDecoder
    {
    public:
        Vp8GStreamerVideoDecoder(std::string decoderPipeline);
        ~Vp8GStreamerVideoDecoder() override = default;

        static const char* codecName();
    };

    class SoftwareVp8GStreamerVideoDecoder : public Vp8GStreamerVideoDecoder
    {
    public:
        SoftwareVp8GStreamerVideoDecoder();
        ~SoftwareVp8GStreamerVideoDecoder() override = default;

        webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
    };

    class VaapiVp8GStreamerVideoDecoder : public Vp8GStreamerVideoDecoder
    {
    public:
        VaapiVp8GStreamerVideoDecoder();
        ~VaapiVp8GStreamerVideoDecoder() override = default;

        webrtc::VideoDecoder::DecoderInfo GetDecoderInfo() const override;

        static bool isSupported();
    };
}

#endif
