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

#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp8GStreamerVideoDecoder.h>

using namespace opentera;
using namespace std;

Vp8GStreamerVideoDecoder::Vp8GStreamerVideoDecoder(string decoderPipeline) :
      GStreamerVideoDecoder("video/x-vp8", move(decoderPipeline))
{
}

const char* Vp8GStreamerVideoDecoder::codecName()
{
    return cricket::kVp8CodecName;
}


SoftwareVp8GStreamerVideoDecoder::SoftwareVp8GStreamerVideoDecoder() :
      Vp8GStreamerVideoDecoder("vp8dec name=decode")
{
}

webrtc::VideoDecoder::DecoderInfo SoftwareVp8GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vp8dec";
    info.is_hardware_accelerated = false;
    return info;
}


VaapiVp8GStreamerVideoDecoder::VaapiVp8GStreamerVideoDecoder() :
      Vp8GStreamerVideoDecoder("vaapivp8dec name=decode ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiVp8GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapivp8dec";
    info.is_hardware_accelerated = true;
    return info;
}
