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

#include <OpenteraWebrtcNativeGStreamer/Codecs/H264GStreamerVideoDecoder.h>

using namespace opentera;
using namespace std;

H264GStreamerVideoDecoder::H264GStreamerVideoDecoder(string decoderPipeline) :
      GStreamerVideoDecoder("video/x-h264", codecName(), move(decoderPipeline))
{
}

const char* H264GStreamerVideoDecoder::codecName()
{
    return cricket::kH264CodecName;
}

SoftwareH264GStreamerVideoDecoder::SoftwareH264GStreamerVideoDecoder() :
      H264GStreamerVideoDecoder("h264parse ! avdec_h264 name=decode")
{
}

webrtc::VideoDecoder::DecoderInfo SoftwareH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer avdec_h264";
    info.is_hardware_accelerated = false;
    return info;
}

VaapiH264GStreamerVideoDecoder::VaapiH264GStreamerVideoDecoder() :
      H264GStreamerVideoDecoder("vaapih264dec name=decode ! vaapipostproc")
{
}

webrtc::VideoDecoder::DecoderInfo VaapiH264GStreamerVideoDecoder::GetDecoderInfo() const
{
    webrtc::VideoDecoder::DecoderInfo info;
    info.implementation_name = "GStreamer vaapih264dec";
    info.is_hardware_accelerated = true;
    return info;
}
