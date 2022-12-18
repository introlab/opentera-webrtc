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

#include <OpenteraWebrtcNativeGStreamer/Codecs/Vp9GStreamerVideoDecoder.h>

using namespace opentera;
using namespace std;

Vp9GStreamerVideoDecoder::Vp9GStreamerVideoDecoder(string decoderPipeline) :
      GStreamerVideoDecoder("video/x-vp9", codecName(), move(decoderPipeline))
{
}

const char* Vp9GStreamerVideoDecoder::codecName()
{
    return cricket::kVp9CodecName;
}

SoftwareVp9GStreamerVideoDecoder::SoftwareVp9GStreamerVideoDecoder() :
      Vp9GStreamerVideoDecoder("vp9dec name=decode")
{
}

webrtc::VideoDecoder::DecoderInfo SoftwareVp9GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vp9dec";
    info.is_hardware_accelerated = false;
    return info;
}

VaapiVp9GStreamerVideoDecoder::VaapiVp9GStreamerVideoDecoder() :
      Vp9GStreamerVideoDecoder("vaapivp9dec name=decode ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiVp9GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapivp9dec";
    info.is_hardware_accelerated = true;
    return info;
}
