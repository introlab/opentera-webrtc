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
